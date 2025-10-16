# AMBA AHB to APB Bridge  (ongoing)
**Design and Verification using SystemVerilog**

## 📘 Overview

This project implements an AMBA AHB to APB bridge, designed and verified in **SystemVerilog** using **ModelSim**. The bridge connects a high-speed AHB master to a low-speed APB slave by translating the protocols.

---

## 📁 Directory Structure

```

.
├── rtl/           # SystemVerilog RTL design
│   └── ahb_apb_bridge.sv
├── tb/            # Testbench files
│   └── testbench.sv
├── sim/           # ModelSim simulation scripts (optional)
└── README.md

````

---

## ▶️ Running Simulation in ModelSim

1. **Open ModelSim terminal** or use `vsim` in your shell.
2. **Compile the files**:

```sh
vlib work
vlog rtl/ahb_apb_bridge.sv tb/testbench.sv
````

3. **Run simulation**:

```sh
vsim testbench
run -all
```

4. **View waveforms (optional):**

```sh
add wave *
```

---

## ✅ Contents

* `rtl/ahb_apb_bridge.sv` — Bridge RTL logic.
* `tb/testbench.sv` — Basic functional testbench.

