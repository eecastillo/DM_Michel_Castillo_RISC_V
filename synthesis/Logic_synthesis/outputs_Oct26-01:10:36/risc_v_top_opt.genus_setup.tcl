################################################################################
#
# Genus(TM) Synthesis Solution setup file
# Created by Genus(TM) Synthesis Solution 20.11-s111_1
#   on 10/26/2023 01:26:54
#
# This file can only be run in Genus Common UI mode.
#
################################################################################


# This script is intended for use with Genus(TM) Synthesis Solution version 20.11-s111_1


# Remove Existing Design
################################################################################
if {[::legacy::find -design design:risc_v_top] ne ""} {
  puts "** A design with the same name is already loaded. It will be removed. **"
  delete_obj design:risc_v_top
}


# To allow user-readonly attributes
################################################################################
::legacy::set_attribute -quiet force_tui_is_remote 1 /


# Source INIT Setup file
################################################################################
source outputs_Oct26-01:10:36/risc_v_top_opt.genus_init.tcl
read_metric -id current outputs_Oct26-01:10:36/risc_v_top_opt.metrics.json

phys::read_script outputs_Oct26-01:10:36/risc_v_top_opt.g
puts "\n** Restoration Completed **\n"


# Data Integrity Check
################################################################################
# program version
if {"[string_representation [::legacy::get_attribute program_version /]]" != "20.11-s111_1"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-91] "golden program_version: 20.11-s111_1  current program_version: [string_representation [::legacy::get_attribute program_version /]]"
}
# license
if {"[string_representation [::legacy::get_attribute startup_license /]]" != "Genus_Synthesis"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-91] "golden license: Genus_Synthesis  current license: [string_representation [::legacy::get_attribute startup_license /]]"
}
# slack
set _slk_ [::legacy::get_attribute slack design:risc_v_top]
if {[regexp {^-?[0-9.]+$} $_slk_]} {
  set _slk_ [format %.1f $_slk_]
}
if {$_slk_ != "126.7"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden slack: 126.7,  current slack: $_slk_"
}
unset _slk_
# multi-mode slack
if {"[string_representation [::legacy::get_attribute slack_by_mode design:risc_v_top]]" != "{{mode:risc_v_top/view_Adders_slow 4082.0} {mode:risc_v_top/view_Adders_fast 126.7}}"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden slack_by_mode: {{mode:risc_v_top/view_Adders_slow 4082.0} {mode:risc_v_top/view_Adders_fast 126.7}}  current slack_by_mode: [string_representation [::legacy::get_attribute slack_by_mode design:risc_v_top]]"
}
# tns
set _tns_ [::legacy::get_attribute tns design:risc_v_top]
if {[regexp {^-?[0-9.]+$} $_tns_]} {
  set _tns_ [format %.0f $_tns_]
}
if {$_tns_ != "0"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden tns: 0,  current tns: $_tns_"
}
unset _tns_
# cell area
set _cell_area_ [::legacy::get_attribute cell_area design:risc_v_top]
if {[regexp {^-?[0-9.]+$} $_cell_area_]} {
  set _cell_area_ [format %.0f $_cell_area_]
}
if {$_cell_area_ != "23537"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden cell area: 23537,  current cell area: $_cell_area_"
}
unset _cell_area_
# net area
set _net_area_ [::legacy::get_attribute net_area design:risc_v_top]
if {[regexp {^-?[0-9.]+$} $_net_area_]} {
  set _net_area_ [format %.0f $_net_area_]
}
if {$_net_area_ != "658"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden net area: 658,  current net area: $_net_area_"
}
unset _net_area_
# library domain count
if {[llength [::legacy::find /libraries -library_domain *]] != "2"} {
   mesg_send [::legacy::find -message /messages/PHYS/PHYS-92] "golden # library domains: 2  current # library domains: [llength [::legacy::find /libraries -library_domain *]]"
}
