library verilog;
use verilog.vl_types.all;
entity traffic_system_top is
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        emergency_sw    : in     vl_logic;
        ns_density_i    : in     vl_logic_vector(3 downto 0);
        ew_density_i    : in     vl_logic_vector(3 downto 0);
        camera_valid_i  : in     vl_logic;
        ped_btn_ns_i    : in     vl_logic;
        ped_btn_ew_i    : in     vl_logic;
        ns_leds         : out    vl_logic_vector(2 downto 0);
        ew_leds         : out    vl_logic_vector(2 downto 0);
        ped_ns_led      : out    vl_logic;
        ped_ew_led      : out    vl_logic;
        ped_ns_req_led_o: out    vl_logic;
        ped_ew_req_led_o: out    vl_logic
    );
end traffic_system_top;
