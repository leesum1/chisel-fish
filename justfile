FISH_CORE_HOME := `pwd`

Linux_payload := FISH_CORE_HOME / "sim/ready_to_run/linux-8m-hello-hvc.elf"
sim_dir := FISH_CORE_HOME / "sim"

clean_dir := "\
project \
.venv \
target \
sim/.xmake \
sim/.cache \
sim/build \
"



default: 
    just --list

gen_fish_soc_verilog:
    cd {{FISH_CORE_HOME}} && sbt "runMain leesum.Core.gen_FishSoc"

gen_fish_core_verilog:
    cd {{FISH_CORE_HOME}} && sbt "runMain leesum.Core.gen_FishCore_verilog"

build_sim: gen_fish_soc_verilog
    cd {{sim_dir}} && xmake

linux: build_sim
    cd {{sim_dir}} && xmake run Vtop --file {{Linux_payload}} --clk 500000000

clean_all:
    rm -rf {{clean_dir}}
