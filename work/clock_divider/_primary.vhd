library verilog;
use verilog.vl_types.all;
entity clock_divider is
    generic(
        FREQ            : integer := 50000000
    );
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        tick_out        : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of FREQ : constant is 1;
end clock_divider;
