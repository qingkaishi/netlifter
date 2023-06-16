

1. l2cap/bluetooth: logical link control and adaptation protocol
    https://elixir.bootlin.com/linux/v5.15-rc1/source/net/bluetooth/l2cap_core.c
    benchmarks/l2cap.bc -popeye-entry=popeye_main_l2cap -popeye-dot=l2cap.dot

2. smp/bluetooth: low energy security manager protocol
    https://elixir.bootlin.com/linux/v5.15-rc1/source/net/bluetooth/smp.c
    benchmarks/smp.bc -popeye-entry=popeye_main_smp -popeye-dot=smp.dot

3. apdu/smartcard: application protocol data unit
    https://github.com/OpenSC/OpenSC
    benchmarks/apdu.bc -popeye-entry=popeye_main_apdu -popeye-dot=apdu.dot

4. osdp: open supervised device protocol
    https://github.com/goToMain/libosdp
    benchmarks/osdp.bc -popeye-entry=popeye_main_osdp -popeye-dot=osdp.dot

5. ssq: source server query protocol
    https://github.com/BinaryAlien/libssq
    benchmarks/ssq.bc -popeye-entry=popeye_main_ssq -popeye-dot=ssq.dot

6. tcp/ip: transport control porotocol
    https://github.com/lwip-tcpip/lwip
    benchmarks/tcp.bc -popeye-enable-global -popeye-entry=popeye_main_ethernet_ip4_tcp -popeye-dot=tcpip4.dot

7. igmp/ip: internet group management protocol
    https://github.com/lwip-tcpip/lwip
    benchmarks/igmp.bc -popeye-enable-global -popeye-entry=popeye_main_ethernet_ip4_igmp -popeye-dot=igmpip4.dot

8. quic: a general-purpose transport layer network protocol
    https://github.com/ngtcp2/ngtcp2
    benchmarks/quic.bc -popeye-entry=popeye_main_hd_long -popeye-dot=quic.dot
    benchmarks/quic.bc -popeye-entry=popeye_main_hd_short -popeye-dot=quic.dot

9. babel: a distance-vector routing protocol that is designed to be robust and efficient on both wireless mesh networks and wired networks
    https://github.com/FRRouting/frr/
    benchmarks/babel.bc -popeye-entry=popeye_main_babel -popeye-dot=babel.dot

10. isis: intermediate system to intermediate system protocol
    https://github.com/FRRouting/frr/
    benchmarks/isis.bc -popeye-entry=popeye_main_isis -popeye-dot=isis.dot

