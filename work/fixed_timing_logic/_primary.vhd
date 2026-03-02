library verilog;
use verilog.vl_types.all;
entity fixed_timing_logic is
    generic(
        T_GREEN         : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi1, Hi0, Hi1, Hi0);
        T_YELLOW        : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi0, Hi1, Hi0, Hi1);
        T_RED           : vl_logic_vector(0 to 5) := (Hi0, Hi0, Hi1, Hi0, Hi1, Hi0);
        S_NS_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi0, Hi1);
        S_NS_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi0, Hi1, Hi0);
        S_EW_GREEN      : vl_logic_vector(0 to 4) := (Hi0, Hi0, Hi1, Hi0, Hi0);
        S_EW_YELLOW     : vl_logic_vector(0 to 4) := (Hi0, Hi1, Hi0, Hi0, Hi0)
    );
    port(
        current_state_i : in     vl_logic_vector(4 downto 0);
        duration_o      : out    vl_logic_vector(5 downto 0)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of T_GREEN : constant is 1;
    attribute mti_svvh_generic_type of T_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of T_RED : constant is 1;
    attribute mti_svvh_generic_type of S_NS_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_NS_YELLOW : constant is 1;
    attribute mti_svvh_generic_type of S_EW_GREEN : constant is 1;
    attribute mti_svvh_generic_type of S_EW_YELLOW : constant is 1;
end fixed_timing_logic;
