@echo off
set xv_path=C:\\Xilinx\\Vivado\\2017.2\\bin
call %xv_path%/xsim sim_behav -key {Behavioral:sim_1:Functional:sim} -tclbatch sim.tcl -view C:/GitHub/XMEGA_CORE_IP_V2/xmega_core_v2_sim_behav.wcfg -log simulate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
