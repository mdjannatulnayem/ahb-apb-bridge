module ahb3lite_apb_bridge
  import ahb3lite_pkg::*;
#(
  parameter HADDR_SIZE = 32,
  parameter HDATA_SIZE = 32,
  parameter PADDR_SIZE = 10,
  parameter PDATA_SIZE = 8,
  parameter SYNC_DEPTH = 3
)
(
  // AHB Slave Interface
  input                         HRESETn,
                                HCLK,
  input                         HSEL,
  input      [HADDR_SIZE-1:0]   HADDR,
  input      [HDATA_SIZE-1:0]   HWDATA,
  output reg [HDATA_SIZE-1:0]   HRDATA,
  input                         HWRITE,
  input      [2:0]              HSIZE,
  input      [2:0]              HBURST,
  input      [3:0]              HPROT,
  input      [1:0]              HTRANS,
  input                         HMASTLOCK,
  output reg                    HREADYOUT,
  input                         HREADY,
  output reg                    HRESP,

  // APB Master Interface
  input                         PRESETn,
                                PCLK,
  output reg                    PSEL,
  output reg                    PENABLE,
  output reg [2:0]              PPROT,
  output reg                    PWRITE,
  output reg [PDATA_SIZE/8-1:0] PSTRB,
  output reg [PADDR_SIZE-1:0]   PADDR,
  output reg [PDATA_SIZE-1:0]   PWDATA,
  input      [PDATA_SIZE-1:0]   PRDATA,
  input                         PREADY,
  input                         PSLVERR
);

  typedef enum logic [1:0] { ST_AHB_IDLE = 2'b00, ST_AHB_TRANSFER = 2'b01, ST_AHB_ERROR = 2'b10 } ahb_fsm_states;
  typedef enum logic [1:0] { ST_APB_IDLE = 2'b00, ST_APB_SETUP    = 2'b01, ST_APB_TRANSFER = 2'b10 } apb_fsm_states;

  localparam [2:0] PPROT_NORMAL      = 3'b000,
                   PPROT_PRIVILEGED  = 3'b001,
                   PPROT_SECURE      = 3'b000,
                   PPROT_NONSECURE   = 3'b010,
                   PPROT_DATA        = 3'b000,
                   PPROT_INSTRUCTION = 3'b100;

  localparam SYNC_DEPTH_MIN = 3;
  localparam SYNC_DEPTH_CHK = SYNC_DEPTH > SYNC_DEPTH_MIN ? SYNC_DEPTH : SYNC_DEPTH_MIN;

  localparam int PSTRB_WIDTH = PDATA_SIZE/8;

  // run-time checks
  initial begin
    a1: assert (HDATA_SIZE % 8 == 0) else $error("HDATA_SIZE must be multiple of bytes (8)");
    a2: assert (PDATA_SIZE % 8 == 0) else $error("PDATA_SIZE must be multiple of bytes (8)");
    a3: assert (PDATA_SIZE <= HDATA_SIZE) else $error("PDATA_SIZE must be <= HDATA_SIZE");
    a4: assert (SYNC_DEPTH >= SYNC_DEPTH_MIN) else $warning("SYNC_DEPTH=%0d < minimum - using %0d", SYNC_DEPTH, SYNC_DEPTH_CHK);
  end

  // signals
  logic                      ahb_treq;
  logic                      treq_toggle;
  logic [SYNC_DEPTH_CHK-1:0] treq_sync;
  logic                      apb_treq_strb;

  logic                      apb_tack;
  logic                      tack_toggle;
  logic [SYNC_DEPTH_CHK-1:0] tack_sync;
  logic                      ahb_tack_strb;

  logic [HADDR_SIZE-1:0]     ahb_haddr;
  logic [HDATA_SIZE-1:0]     ahb_hwdata;
  logic                      ahb_hwrite;
  logic [2:0]                ahb_hsize;
  logic [3:0]                ahb_hprot;

  logic                      latch_ahb_hwdata;

  logic [HDATA_SIZE-1:0]     apb_prdata;
  logic                      apb_pslverr;

  ahb_fsm_states             ahb_fsm;
  apb_fsm_states             apb_fsm;

  logic [6:0]                apb_beat_cnt;
  logic [9:0]                apb_beat_data_offset;

  // small helper tasks
  task automatic ahb_no_transfer();
    ahb_fsm   <= ST_AHB_IDLE;
    HREADYOUT <= 1'b1;
    HRESP     <= HRESP_OKAY;
  endtask

  task automatic ahb_prep_transfer();
    ahb_fsm   <= ST_AHB_TRANSFER;
    HREADYOUT <= 1'b0;
    HRESP     <= HRESP_OKAY;
    ahb_treq  <= 1'b1;
  endtask

  function automatic logic [6:0] apb_beats(input [2:0] hsize);
    case (hsize)
      HSIZE_B1024: apb_beats = 1023/PDATA_SIZE;
      HSIZE_B512 : apb_beats = 511/PDATA_SIZE;
      HSIZE_B256 : apb_beats = 255/PDATA_SIZE;
      HSIZE_B128 : apb_beats = 127/PDATA_SIZE;
      HSIZE_DWORD: apb_beats = 63/PDATA_SIZE;
      HSIZE_WORD : apb_beats = 31/PDATA_SIZE;
      HSIZE_HWORD: apb_beats = 15/PDATA_SIZE;
      default    : apb_beats = 7/PDATA_SIZE;
    endcase
  endfunction

  function automatic logic [6:0] address_mask(input integer data_size);
    case (data_size)
      1024: address_mask = 7'b111_1111;
      512 : address_mask = 7'b011_1111;
      256 : address_mask = 7'b001_1111;
      128 : address_mask = 7'b000_1111;
      64  : address_mask = 7'b000_0111;
      32  : address_mask = 7'b000_0011;
      16  : address_mask = 7'b000_0001;
      default: address_mask = 7'b000_0000;
    endcase
  endfunction

  function automatic logic [9:0] data_offset(input [HADDR_SIZE-1:0] haddr);
    logic [6:0] haddr_masked;
    haddr_masked = haddr & address_mask(HDATA_SIZE);
    data_offset = 8 * haddr_masked;
  endfunction

  function automatic logic [PSTRB_WIDTH-1:0] pstrb(input [2:0] hsize, input [PADDR_SIZE-1:0] paddr);
    logic [127:0] full_pstrb;
    logic [6:0]   paddr_masked;

    case (hsize)
      HSIZE_B1024: full_pstrb = {128{1'b1}};
      HSIZE_B512 : full_pstrb = {64{1'b1}};
      HSIZE_B256 : full_pstrb = {32{1'b1}};
      HSIZE_B128 : full_pstrb = {16{1'b1}};
      HSIZE_DWORD: full_pstrb = {8{1'b1}};
      HSIZE_WORD : full_pstrb = {4{1'b1}};
      HSIZE_HWORD: full_pstrb = {2{1'b1}};
      default    : full_pstrb = {1{1'b1}};
    endcase

    paddr_masked = paddr & address_mask(PDATA_SIZE);
    pstrb = full_pstrb[PSTRB_WIDTH-1:0] << paddr_masked;
  endfunction

  // AHB FSM
  always_ff @(posedge HCLK or negedge HRESETn)
    if (!HRESETn) begin
      ahb_fsm    <= ST_AHB_IDLE;
      HREADYOUT  <= 1'b1;
      HRESP      <= HRESP_OKAY;
      HRDATA     <= 'hx;
      ahb_treq   <= 1'b0;
      ahb_haddr  <= 'h0;
      ahb_hwrite <= 1'b0;
      ahb_hprot  <= 'h0;
      ahb_hsize  <= 'h0;
    end else begin
      ahb_treq <= 1'b0;

      case (ahb_fsm)
        ST_AHB_IDLE: begin
          ahb_haddr  <= HADDR;
          ahb_hwrite <= HWRITE;
          ahb_hprot  <= HPROT;
          ahb_hsize  <= HSIZE;

          if (HSEL && HREADY) begin
            case (HTRANS)
              HTRANS_IDLE, HTRANS_BUSY: ahb_no_transfer();
              HTRANS_NONSEQ, HTRANS_SEQ: ahb_prep_transfer();
              default: ahb_no_transfer();
            endcase
          end else begin
            ahb_no_transfer();
          end
        end

        ST_AHB_TRANSFER:
          if (ahb_tack_strb) begin
            HRDATA <= apb_prdata;
            if (apb_pslverr) begin
              HREADYOUT <= 1'b0;
              HRESP     <= HRESP_ERROR;
              ahb_fsm   <= ST_AHB_ERROR;
            end else begin
              HREADYOUT <= 1'b1;
              HRESP     <= HRESP_OKAY;
              ahb_fsm   <= ST_AHB_IDLE;
            end
          end else begin
            HREADYOUT <= 1'b0;
          end

        ST_AHB_ERROR: begin
          ahb_fsm   <= ST_AHB_IDLE;
          HREADYOUT <= 1'b1;
        end
      endcase
    end

  always_ff @(posedge HCLK)
    latch_ahb_hwdata <= HSEL & HREADY & HWRITE & ((HTRANS == HTRANS_NONSEQ) || (HTRANS == HTRANS_SEQ));

  always_ff @(posedge HCLK)
    if (latch_ahb_hwdata) ahb_hwdata <= HWDATA;

  // cross-domain request toggle (AHB -> APB)
  always_ff @(posedge HCLK or negedge HRESETn)
    if (!HRESETn) treq_toggle <= 1'b0;
    else if (ahb_treq) treq_toggle <= ~treq_toggle;

  always_ff @(posedge PCLK or negedge PRESETn)
    if (!PRESETn) treq_sync <= 'h0;
    else treq_sync <= { treq_sync[SYNC_DEPTH_CHK-2:0], treq_toggle };

  assign apb_treq_strb = treq_sync[SYNC_DEPTH_CHK-1] ^ treq_sync[SYNC_DEPTH_CHK-2];

  // cross-domain ack toggle (APB -> AHB)
  always_ff @(posedge PCLK or negedge PRESETn)
    if (!PRESETn) tack_toggle <= 1'b0;
    else if (apb_tack) tack_toggle <= ~tack_toggle;

  always_ff @(posedge HCLK or negedge HRESETn)
    if (!HRESETn) tack_sync <= 'h0;
    else tack_sync <= { tack_sync[SYNC_DEPTH_CHK-2:0], tack_toggle };

  assign ahb_tack_strb = tack_sync[SYNC_DEPTH_CHK-1] ^ tack_sync[SYNC_DEPTH_CHK-2];

  // APB FSM (on PCLK)
  always_ff @(posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      apb_fsm              <= ST_APB_IDLE;
      apb_tack             <= 1'b0;
      apb_prdata           <= 'hx;
      apb_beat_cnt         <= 'hx;
      apb_beat_data_offset <= 'hx;
      apb_pslverr          <= 1'bx;

      PSEL   <= 1'b0;
      PPROT  <= 1'b0;
      PADDR  <= 'h0;
      PWRITE <= 1'b0;
      PENABLE<= 1'b0;
      PWDATA <= 'h0;
      PSTRB  <= 'h0;
    end else begin
      apb_tack <= 1'b0;

      case (apb_fsm)
        ST_APB_IDLE:
          if (apb_treq_strb) begin
            apb_fsm    <= ST_APB_SETUP;
            PSEL       <= 1'b1;
            PENABLE    <= 1'b0;
            PPROT      <= ((ahb_hprot & HPROT_DATA      ) ? PPROT_DATA       : PPROT_INSTRUCTION) |
                          ((ahb_hprot & HPROT_PRIVILEGED) ? PPROT_PRIVILEGED : PPROT_NORMAL);
            PADDR      <= ahb_haddr[PADDR_SIZE-1:0];
            PWRITE     <= ahb_hwrite;
            PWDATA     <= ahb_hwdata >> data_offset(ahb_haddr);
            PSTRB      <= {PSTRB_WIDTH{ahb_hwrite}} & pstrb(ahb_hsize, ahb_haddr[PADDR_SIZE-1:0]);

            apb_prdata           <= 'h0;
            apb_beat_cnt         <= apb_beats(ahb_hsize);
            apb_beat_data_offset <= data_offset(ahb_haddr) + PDATA_SIZE;
          end

        ST_APB_SETUP: begin
          apb_fsm <= ST_APB_TRANSFER;
          PENABLE <= 1'b1;
        end

        ST_APB_TRANSFER:
          if (PREADY) begin
            apb_beat_cnt         <= apb_beat_cnt - 1;
            apb_beat_data_offset <= apb_beat_data_offset + PDATA_SIZE;

            apb_prdata           <= (apb_prdata << PDATA_SIZE) | (PRDATA << data_offset(ahb_haddr));
            apb_pslverr          <= PSLVERR;

            PENABLE <= 1'b0;

            if (PSLVERR || ~|apb_beat_cnt) begin
              apb_fsm  <= ST_APB_IDLE;
              apb_tack <= 1'b1;
              PSEL     <= 1'b0;
            end else begin
              apb_fsm <= ST_APB_SETUP;
              PADDR   <= PADDR + PSTRB_WIDTH; // increment by number of bytes in PDATA
              PWDATA  <= ahb_hwdata >> apb_beat_data_offset;
              PSTRB   <= {PSTRB_WIDTH{ahb_hwrite}} & pstrb(ahb_hsize, PADDR + PSTRB_WIDTH);
            end
          end
      endcase
    end

endmodule
