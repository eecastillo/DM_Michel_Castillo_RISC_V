tclmode

# Generated by Genus(TM) Synthesis Solution 20.11-s111_1, revision 1.483c0
# Generated on: Thu Oct 26 04:19:36 UTC 2023 (Thu Oct 26 04:19:36 UTC 2023)

set_screen_display -noprogress
# Argument '-no_exit' was specified.
#set_dofile_abort exit

### Alias mapping flow is enabled. ###
# Root attribute 'wlec_rtl_name_mapping_flow' was 'false'.
# Root attribute 'alias_flow'                 was 'true'.

set lec_version [regsub {(-)[A-Za-z]} $env(LEC_VERSION) ""]

# Turns on the flowgraph datapath solver.
set wlec_analyze_dp_flowgraph true
# Indicates that resource sharing datapath optimization is present.
set share_dp_analysis         true

# The flowgraph solver is recommended for datapath analysis in LEC 19.1 or newer.
set lec_version_required "19.10100"
if {$lec_version >= $lec_version_required &&
    $wlec_analyze_dp_flowgraph} {
    set DATAPATH_SOLVER_OPTION "-flowgraph"
} elseif {$share_dp_analysis} {
    set DATAPATH_SOLVER_OPTION "-share"
} else {
    set DATAPATH_SOLVER_OPTION ""
}

# This composite dofile includes two compare operations: rtl-to-fv_map and
# fv_map-to-revised. The 'fv_map' netlist was automatically written in the
# verification directory during the syn_map command.

# This function is only valid for a flat compare
proc is_pass {} {
    redirect -variable compare_result {report_verification}
    foreach i [split $compare_result "\n"] {
        if {[regexp {Compare Results:\s+PASS} $i]} {
            return true
        }
    }
    return false
}

tcl_set_command_name_echo on

set_log_file logs_Oct26-04:16:26/rtl2intermediate.lec.log -replace

usage -auto -elapse

# Comparing intermediate 'fv_map' netlist vs. revised netlist.

set_verification_information fv_map_risc_v_top_intermediatev_db

read_implementation_information fv/risc_v_top -golden fv_map -revised risc_v_top_intermediatev

# Root attribute 'wlec_multithread_license_list' can be used to specify a license list
# for multithreaded processing. The default list is used otherwise.
set_parallel_option -threads 1,4 -norelease_license
set_compare_options -threads 1,4

set env(RC_VERSION)     "20.11-s111_1"
set env(CDN_SYNTH_ROOT) "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86"
set CDN_SYNTH_ROOT      "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86"
set env(CW_DIR) "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86/lib/chipware"
set CW_DIR      "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86/lib/chipware"

# default is to error out when module definitions are missing
set_undefined_cell black_box -noascend -both

add_search_path /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/ /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/lef/ -library -both
read_library -liberty -both /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/slow_vdd1v0_basicCells.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/slow_vdd1v0_multibitsDFF.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/fast_vdd1v2_basicCells.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/fast_vdd1v2_multibitsDFF.lib

read_design -verilog95   -golden -lastmod -noelab fv/risc_v_top/fv_map.v.gz
elaborate_design -golden -root {risc_v_top}

read_design -verilog95   -revised -lastmod -noelab Netlist/risc_v_top_intermediate.v
elaborate_design -revised -root {risc_v_top}

#add_name_alias fv/risc_v_top/fv_map.singlebit.original_name.alias.json.gz -golden
#set_mapping_method -alias -golden

report_design_data
report_black_box

set_flatten_model -seq_constant
set_flatten_model -seq_constant_x_to 0
set_flatten_model -nodff_to_dlat_zero
set_flatten_model -nodff_to_dlat_feedback
set_flatten_model -hier_seq_merge

#add_name_alias fv/risc_v_top/risc_v_top_intermediatev.singlebit.original_name.alias.json.gz -revised
#set_mapping_method -alias -revised
#add_renaming_rule r1alias _reg((\\\[%w\\\])*(/U\\\$%d)*)$ @1 -type dff dlat -both

# Reports the quality of the implementation information.
# This command is only available with LEC 20.10-p100 or later.
set lec_version_required "20.10100"
if {$lec_version >= $lec_version_required} {
    check_verification_information
}

set_analyze_option -auto -report_map

set_system_mode lec
report_unmapped_points -summary
report_unmapped_points -notmapped
add_compared_points -all
compare

report_compare_data -class nonequivalent -class abort -class notcompared
report_verification -verbose
report_statistics

# Check intermediate 'fv_map' netlist vs. revised netlist compare result
if {![is_pass]} {
    error "// ERROR: Compare was not equivalent."
}

write_verification_information
report_verification_information

# Reports how effective the implementation information was.
# This command is only available with LEC 18.20-d330 or later.
set lec_version_required "18.20330"
if {$lec_version >= $lec_version_required} {
    report_implementation_information
}

reset

# Comparing RTL vs. intermediate 'fv_map' netlist.

set_verification_information rtl_fv_map_db

read_implementation_information fv/risc_v_top -revised fv_map

# Root attribute 'wlec_multithread_license_list' can be used to specify a license list
# for multithreaded processing. The default list is used otherwise.
set_parallel_option -threads 1,4 -norelease_license
set_compare_options -threads 1,4

set env(RC_VERSION)     "20.11-s111_1"
set env(CDN_SYNTH_ROOT) "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86"
set CDN_SYNTH_ROOT      "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86"
set env(CW_DIR) "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86/lib/chipware"
set CW_DIR      "/CMC/tools/cadence/GENUS20.11.000_lnx86/tools.lnx86/lib/chipware"

# default is to error out when module definitions are missing
set_undefined_cell black_box -noascend -both

add_search_path /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/ /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/lef/ -library -both
read_library -liberty -both /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/slow_vdd1v0_basicCells.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/slow_vdd1v0_multibitsDFF.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/fast_vdd1v2_basicCells.lib /CMC/kits/cadence/GPDK045/gsclib045_all_v4.4/gsclib045/timing/fast_vdd1v2_multibitsDFF.lib

set_undriven_signal 0 -golden
set lec_version_required "16.20100"
if {$lec_version >= $lec_version_required} {
    set_naming_style genus -golden
} else {
    set_naming_style rc -golden
}
set_naming_rule "%s\[%d\]" -instance_array -golden
set_naming_rule "%s_reg" -register -golden
set_naming_rule "%L.%s" "%L\[%d\].%s" "%s" -instance -golden
set_naming_rule "%L.%s" "%L\[%d\].%s" "%s" -variable -golden
set lec_version_required "17.10200"
if {$lec_version >= $lec_version_required} {
    set_naming_rule -ungroup_separator {_} -golden
}

# Align LEC's treatment of mismatched port widths with constant
# connections with Genus's. Genus message CDFG-467 and LEC message
# HRC3.6 may indicate the presence of this issue.
# This option is only available with LEC 17.20-d301 or later.
set lec_version_required "17.20301"
if {$lec_version >= $lec_version_required} {
    set_hdl_options -const_port_extend
}

set_hdl_options -VERILOG_INCLUDE_DIR "sep:src:cwd"
delete_search_path -all -design -golden
add_search_path ../../src -design -golden
read_design  -define SYNTHESIS  -merge bbox -golden -lastmod -noelab -verilog2k  ../../src/Counter_02Limit_ovf.v ../../src/risc_v_top.v ../../src/multiplexor_param.v ../../src/ffd_param_pc_risk.v ../../src/adder.v ../../src/branch_prediction.v ../../src/instr_memory.v ../../src/ffd_param_clear_n.v ../../src/imm_gen.v ../../src/register_file.v ../../src/jump_detection_unit.v ../../src/control_unit.v ../../src/hazard_detection_unit.v ../../src/ALU_control.v ../../src/double_multiplexor_param.v ../../src/ALU.v ../../src/forward_unit.v ../../src/master_memory_map.v ../../src/data_memory.v ../../src/uart_IP.v ../../src/branch_control_unit.v ../../src/uart_full_duplex.v ../../src/UART_Rx.v ../../src/uart_tx.v ../../src/uart_tx_fsm.v ../../src/ffd_param_clear.v ../../src/Delayer.v ../../src/Bit_Rate_Pulse.v ../../src/ffd_param.v ../../src/parallel2serial.v ../../src/Shift_Register_R_Param.v ../../src/FSM_UART_Rx.v ../../src/Counter_Param.v
elaborate_design -golden -root {risc_v_top} -rootonly 

read_design -verilog95   -revised -lastmod -noelab fv/risc_v_top/fv_map.v.gz
elaborate_design -revised -root {risc_v_top}

uniquify -all -nolib -golden

report_design_data
report_black_box

set_flatten_model -seq_constant
set_flatten_model -seq_constant_x_to 0
set_flatten_model -nodff_to_dlat_zero
set_flatten_model -nodff_to_dlat_feedback
set_flatten_model -hier_seq_merge

set_flatten_model -balanced_modeling

#add_name_alias fv/risc_v_top/fv_map.singlebit.original_name.alias.json.gz -revised
#set_mapping_method -alias -revised
#add_renaming_rule r1alias _reg((\\\[%w\\\])*(/U\\\$%d)*)$ @1 -type dff dlat -both

# Reports the quality of the implementation information.
# This command is only available with LEC 20.10-p100 or later.
set lec_version_required "20.10100"
if {$lec_version >= $lec_version_required} {
    check_verification_information
}

set_analyze_option -auto -report_map

write_hier_compare_dofile hier_tmp2.lec.do -verbose -noexact_pin_match -constraint -usage \
-replace -balanced_extraction -input_output_pin_equivalence \
-prepend_string "report_design_data; report_unmapped_points -summary; report_unmapped_points -notmapped; analyze_datapath -module -verbose; eval analyze_datapath $DATAPATH_SOLVER_OPTION -verbose"
run_hier_compare hier_tmp2.lec.do -dynamic_hierarchy

report_hier_compare_result -dynamicflattened

report_verification -hier -verbose

write_verification_information
report_verification_information

# Reports how effective the implementation information was.
# This command is only available with LEC 18.20-d330 or later.
set lec_version_required "18.20330"
if {$lec_version >= $lec_version_required} {
    report_implementation_information
}

set_system_mode lec

puts "No of compare points = [get_compare_points -count]"
puts "No of diff points    = [get_compare_points -NONequivalent -count]"
puts "No of abort points   = [get_compare_points -abort -count]"
puts "No of unknown points = [get_compare_points -unknown -count]"
if {[get_compare_points -count] == 0} {
    puts "---------------------------------"
    puts "ERROR: No compare points detected"
    puts "---------------------------------"
}
if {[get_compare_points -NONequivalent -count] > 0} {
    puts "------------------------------------"
    puts "ERROR: Different Key Points detected"
    puts "------------------------------------"
}
if {[get_compare_points -abort -count] > 0} {
    puts "-----------------------------"
    puts "ERROR: Abort Points detected "
    puts "-----------------------------"
}
if {[get_compare_points -unknown -count] > 0} {
    puts "----------------------------------"
    puts "ERROR: Unknown Key Points detected"
    puts "----------------------------------"
}

# Generate a detailed summary of the run.
# This command is available with LEC 19.10-p100 or later.
set lec_version_required "19.10100"
if {$lec_version >= $lec_version_required} {
analyze_results -logfiles logs_Oct26-04:16:26/rtl2intermediate.lec.log
}

vpxmode
