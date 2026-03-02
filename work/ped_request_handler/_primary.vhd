library verilog;
use verilog.vl_types.all;
entity ped_request_handler is
    generic(
        CLK_FREQ        : integer := 50000000;
        DEBOUNCE_MS     : integer := 20
    );
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        current_state_i : in     vl_logic_vector(4 downto 0);
        ped_btn_ns_i    : in     vl_logic;
        ped_btn_ew_i    : in     vl_logic;
        ped_ns_req_o    : out    vl_logic;
        ped_ew_req_o    : out    vl_logic;
        ped_ns_req_led_o: out    vl_logic;
        ped_ew_req_led_o: out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of CLK_FREQ : constant is 1;
    attribute mti_svvh_generic_type of DEBOUNCE_MS : constant is 1;
end ped_request_handler;
