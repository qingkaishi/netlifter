popeye_bin=$1
bc_dir=$2
benchmarks_bin_dir=$3

function run_with_entry_name() {
  start_time=$(date +%s)
  proj=$1
  entry=$2
  extra_options=${*:3:100}
  printf "[INFO] Running %8s " $proj
  if [ -f "$bc_dir/$proj.bc" ]; then
	  echo "$popeye_bin $bc_dir/$proj.bc -popeye-entry=$entry $extra_options -popeye-output=dot:$benchmarks_bin_dir/$proj.dot" >$benchmarks_bin_dir/$proj.log
  else
	  echo "$bc_dir/$proj.bc not found.  Skip!"
	  return 0
  fi
  echo "$popeye_bin $bc_dir/$proj.bc -popeye-entry=$entry $extra_options -popeye-output=dot:$benchmarks_bin_dir/$proj.dot" >$benchmarks_bin_dir/$proj.log
  $popeye_bin $bc_dir/$proj.bc -popeye-entry=$entry $extra_options -popeye-output=bnf -popeye-output=dot:$benchmarks_bin_dir/$proj.dot >>$benchmarks_bin_dir/$proj.log 2>$benchmarks_bin_dir/$proj.err
  retcode=$?
  end_time=$(date +%s)
  elapsed=$((end_time - start_time))
  if [ $retcode -ne 0 ]; then
    echo -e "takes $elapsed seconds. Fail!"
    return 1
  else
    printf "takes %5s seconds. " "$elapsed"
  fi

  total_productions=$(grep "# Productions: " $benchmarks_bin_dir/$proj.log)
  printf "%-22s" "$total_productions"

  total_assertions=$(grep "# Assertions: " $benchmarks_bin_dir/$proj.log)
  printf "%-22s" "$total_assertions"

  testfail=$(grep "Test failed " $benchmarks_bin_dir/$proj.log)
  if [ $? -eq 0 ]; then
    printf "%s Fail!\n" "$testfail"
    return 1
  fi

  echo -e "\t Pass!"
  return 0
}

function run() {
  start_time=$(date +%s)
  proj=$1
  extra_options=${*:2:100}

  run_with_entry_name $proj popeye_main_$proj $extra_options
  return $?
}

echo "[INFO] ----------------------------------------------------"
echo "[INFO] Regression begins"
echo "[INFO] ----------------------------------------------------"

rm -f $benchmarks_bin_dir/*.dot
rm -f $benchmarks_bin_dir/*.png
rm -f $benchmarks_bin_dir/*.log
rm -f $benchmarks_bin_dir/*.err

# apdu/smartcard: application protocol data unit
run apdu -popeye-enable-length-solution=false -popeye-oracle-production-num=19 -popeye-oracle-assertion-num=48

# smp/bluetooth: low energy security manager protocol
run smp -popeye-oracle-production-num=25 -popeye-oracle-assertion-num=96

# ssq: source server query protocol
run ssq -popeye-enable-loop-summary -popeye-oracle-production-num=120 -popeye-oracle-assertion-num=741

# l2cap/bluetooth: logical link control and adaptation protocol
run l2cap

# osdp: open supervised device protocol
run osdp

# tcp/ip: transport control porotocol
run_with_entry_name tcp popeye_main_ethernet_ip4_tcp -popeye-enable-global

# igmp/ip: internet group management protocol
run_with_entry_name igmp popeye_main_ethernet_ip4_igmp -popeye-enable-global

# quic: a general-purpose transport layer network protocol
run_with_entry_name quic popeye_main_hd_long

# babel: a distance-vector routing protocol that is designed to be robust and efficient on both wireless mesh networks and wired networks
run babel

# isis: intermediate system to intermediate system protocol
run isis

echo "[INFO] ----------------------------------------------------"
echo "[INFO] Regression completes"
echo "[INFO] ----------------------------------------------------"
