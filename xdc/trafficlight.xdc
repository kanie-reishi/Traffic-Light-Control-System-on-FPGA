## Clock signal
set_property PACKAGE_PIN W5 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Switches
# SW0: emergency_sw
set_property PACKAGE_PIN V17 [get_ports {emergency_sw}]
set_property IOSTANDARD LVCMOS33 [get_ports {emergency_sw}]
# SW15: rst_n (using highest switch, active low: normal run = switch UP, reset = switch DOWN)
set_property PACKAGE_PIN R2 [get_ports {rst_n}]
set_property IOSTANDARD LVCMOS33 [get_ports {rst_n}]

## LEDs
# North-South Traffic Lights (LED 0-2)
set_property PACKAGE_PIN U16 [get_ports {ns_leds[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ns_leds[0]}]
set_property PACKAGE_PIN E19 [get_ports {ns_leds[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ns_leds[1]}]
set_property PACKAGE_PIN U19 [get_ports {ns_leds[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ns_leds[2]}]

# East-West Traffic Lights (LED 3-5)
set_property PACKAGE_PIN V19 [get_ports {ew_leds[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ew_leds[0]}]
set_property PACKAGE_PIN W18 [get_ports {ew_leds[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ew_leds[1]}]
set_property PACKAGE_PIN U15 [get_ports {ew_leds[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {ew_leds[2]}]

# Pedestrian Walk LEDs (LED 6-7)
set_property PACKAGE_PIN U14 [get_ports {ped_ns_led}]
set_property IOSTANDARD LVCMOS33 [get_ports {ped_ns_led}]
set_property PACKAGE_PIN V14 [get_ports {ped_ew_led}]
set_property IOSTANDARD LVCMOS33 [get_ports {ped_ew_led}]

## 7-Segment Display
set_property PACKAGE_PIN W7 [get_ports {seg_o[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[0]}]
set_property PACKAGE_PIN W6 [get_ports {seg_o[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[1]}]
set_property PACKAGE_PIN U8 [get_ports {seg_o[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[2]}]
set_property PACKAGE_PIN V8 [get_ports {seg_o[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[3]}]
set_property PACKAGE_PIN U5 [get_ports {seg_o[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[4]}]
set_property PACKAGE_PIN V5 [get_ports {seg_o[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[5]}]
set_property PACKAGE_PIN U7 [get_ports {seg_o[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg_o[6]}]

set_property PACKAGE_PIN U2 [get_ports {an_o[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an_o[0]}]
set_property PACKAGE_PIN U4 [get_ports {an_o[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an_o[1]}]
set_property PACKAGE_PIN V4 [get_ports {an_o[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an_o[2]}]
set_property PACKAGE_PIN W4 [get_ports {an_o[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an_o[3]}]

## USB-RS232 Interface
# rs_rx mapped to uart_rx
set_property PACKAGE_PIN B18 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
