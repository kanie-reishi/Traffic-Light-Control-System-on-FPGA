library verilog;
use verilog.vl_types.all;
entity adaptive_timing_logic is
    generic(
        T_GREEN_MIN     : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi1, Hi0, Hi0, Hi0);
        T_GREEN_BASE    : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi1, Hi1, Hi1, Hi1);
        T_GREEN_MAX     : vl_logic_vector(0 to 5) := (Hi1, Hi0, Hi1, Hi1, Hi0, Hi1);
        T_YELLOW_FIXED  : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi0, Hi1, Hi0, Hi1);
        T_GREEN_STEP    : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi0, Hi0, Hi1, Hi0);
        MAX_BONUS       : vl_logic_vector(0 to 5) := (Hi0, Hi1, Hi1, Hi1, Hi1, Hi0);
        STARVATION_LIMIT: vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi0, Hi1, Hi0);
        MAX_CONSEC_BONUS: vl_logic_vector(0 to 1) := (Hi1, Hi0);
        S_NS_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi0, Hi1);
        S_NS_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi1, Hi0);
        S_EW_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi1, Hi0, Hi0);
        S_EW_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi1, Hi0, Hi0, Hi0);
        S_ERROR         : vl_logic_vector(0 to 4) := (Hi1, Hi0, Hi0, Hi0, Hi0)
    );
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        tick_1hz        : in     vl_logic;
        next_state_i    : in     vl_logic_vector(4 downto 0);
        current_state_i : in     vl_logic_vector(4 downto 0);
        ns_density_i    : in     vl_logic_vector(3 downto 0);
        ew_density_i    : in     vl_logic_vector(3 downto 0);
        camera_valid_i  : in     vl_logic;
        duration_o      : out    vl_logic_vector(5 downto 0)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of T_GREEN_MIN : constant is 1;
    attribute mti_svvh_generic_type of T_GREEN_BASE : constant is 1;
    attribute mti_svvh_generic_type of T_GREEN_MAX : constant is 1;
    attribute mti_svvh_generic_type of T_YELLOW_FIXED : constant is 1;
    attribute mti_svvh_generic_type of T_GREEN_STEP : constant is 1;
    attribute mti_svvh_generic_type of MAX_BONUS : constant is 1;
    attribute mti_svvh_generic_type of STARVATION_LIMIT : constant is 1;
    attribute mti_svvh_generic_type of MAX_CONSEC_BONUS : constant is 1;
    attribute mti_svvh_generic_type of S_NS_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_NS_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of S_EW_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_EW_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of S_ERROR : constant is 1;
end adaptive_timing_logic;
