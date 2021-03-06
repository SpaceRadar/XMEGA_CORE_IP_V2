#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2017.3 (64-bit)
#
# Filename    : simulate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for simulating the design by launching the simulator
#
# Generated by Vivado on Fri Nov 03 03:06:30 EET 2017
# SW Build 2018833 on Wed Oct  4 19:58:07 MDT 2017
#
# Copyright 1986-2017 Xilinx, Inc. All Rights Reserved.
#
# usage: simulate.sh
#
# ****************************************************************************
ExecStep()
{
"$@"
RETVAL=$?
if [ $RETVAL -ne 0 ]
then
exit $RETVAL
fi
}
ExecStep xsim top_uc_behav -key {Behavioral:sim_1:Functional:top_uc} -tclbatch top_uc.tcl -view /home/w/Documents/GitHub/XMEGA_CORE_IP_V2/xmega_core_v2_sim_behav.wcfg -log simulate.log
