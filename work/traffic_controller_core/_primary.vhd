library verilog;
use verilog.vl_types.all;
entity traffic_controller_core is
    generic(
        S_NS_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi0, Hi1);
        S_NS_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi1, Hi0);
        S_EW_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi1, Hi0, Hi0);
        S_EW_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi1, Hi0, Hi0, Hi0);
        S_ERROR         : vl_logic_vector(0 to 4) := (Hi1, Hi0, Hi0, Hi0, Hi0)
    );
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        emergency_mode  : in     vl_logic;
        tick_1hz        : in     vl_logic;
        req_duration_i  : in     vl_logic_vector(5 downto 0);
        current_state_o : out    vl_logic_vector(4 downto 0);
        next_state_o    : out    vl_logic_vector(4 downto 0);
        ns_light_o      : out    vl_logic_vector(2 downto 0);
        ew_light_o      : out    vl_logic_vector(2 downto 0);
        ped_ns_walk_o   : out    vl_logic;
        ped_ew_walk_o   : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of S_NS_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_NS_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of S_EW_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_EW_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of S_ERROR : constant is 1;
end traffic_controller_core;
