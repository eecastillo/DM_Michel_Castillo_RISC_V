tclmode

# Generated by Genus(TM) Synthesis Solution 20.11-s111_1, revision 1.483c0
# Generated on: Wed Nov 01 03:00:09 UTC 2023 (Wed Nov 01 03:00:09 UTC 2023)

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

tcl_set_command_name_echo on

set_log_file logs_Nov01-02:55:10/intermediate2final.lec.log -replace

usage -auto -elapse

set_verification_information risc_v_top_intermediatev_risc_v_top_optv_db

read_implementation_information fv/risc_v_top -golden risc_v_top_intermediatev -revised risc_v_top_optv

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

read_design -verilog95   -golden -lastmod -noelab Netlist/risc_v_top_intermediate.v
elaborate_design -golden -root {risc_v_top}

read_design -verilog95   -revised -lastmod -noelab Netlist/risc_v_top_opt.v
elaborate_design -revised -root {risc_v_top}

#add_name_alias fv/risc_v_top/risc_v_top_intermediatev.singlebit.original_name.alias.json.gz -golden
#set_mapping_method -alias -golden

report_design_data
report_black_box

set_flatten_model -seq_constant
set_flatten_model -seq_constant_x_to 0
set_flatten_model -nodff_to_dlat_zero
set_flatten_model -nodff_to_dlat_feedback
set_flatten_model -hier_seq_merge

#add_name_alias fv/risc_v_top/risc_v_top_optv.singlebit.original_name.alias.json.gz -revised
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

write_verification_information
report_verification_information

# Reports how effective the implementation information was.
# This command is only available with LEC 18.20-d330 or later.
set lec_version_required "18.20330"
if {$lec_version >= $lec_version_required} {
    report_implementation_information
}

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
analyze_results -logfiles logs_Nov01-02:55:10/intermediate2final.lec.log
}

vpxmode
