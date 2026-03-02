library verilog;
use verilog.vl_types.all;
entity tb_traffic_system is
    generic(
        RED             : vl_logic_vector(0 to 2) := (Hi1, Hi0, Hi0);
        YELLOW          : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi0);
        GREEN           : vl_logic_vector(0 to 2) := (Hi0, Hi0, Hi1)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of RED : constant is 1;
    attribute mti_svvh_generic_type of YELLOW : constant is 1;
    attribute mti_svvh_generic_type of GREEN : constant is 1;
end tb_traffic_system;
