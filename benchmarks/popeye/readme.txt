

1. l2cap/bluetooth: logical link control and adaptation protocol
    https://elixir.bootlin.com/linux/v5.15-rc1/source/net/bluetooth/l2cap_core.c
    benchmarks/l2cap.bc -popeye-entry=popeye_main_l2cap -popeye-output=dot:l2cap.dot -popeye-output=bnf

2. smp/bluetooth: low energy security manager protocol
    https://elixir.bootlin.com/linux/v5.15-rc1/source/net/bluetooth/smp.c
    benchmarks/smp.bc -popeye-entry=popeye_main_smp -popeye-output=dot:smp.dot -popeye-output=bnf

3. apdu/smartcard: application protocol data unit
    https://github.com/OpenSC/OpenSC
    benchmarks/apdu.bc -popeye-entry=popeye_main_apdu -popeye-output=dot:apdu.dot -popeye-output=bnf

4. osdp: open supervised device protocol
    https://github.com/goToMain/libosdp
    benchmarks/osdp.bc -popeye-entry=popeye_main_osdp -popeye-output=dot:osdp.dot -popeye-output=bnf

5. ssq: source server query protocol
    https://github.com/BinaryAlien/libssq
    benchmarks/ssq.bc -popeye-entry=popeye_main_ssq -popeye-output=dot:ssq.dot -popeye-output=bnf

6. tcp/ip: transport control porotocol
    https://github.com/lwip-tcpip/lwip
    benchmarks/tcp.bc -popeye-enable-global -popeye-entry=popeye_main_ethernet_ip4_tcp -popeye-output=dot:tcpip4.dot -popeye-output=bnf

7. igmp/ip: internet group management protocol
    https://github.com/lwip-tcpip/lwip
    benchmarks/igmp.bc -popeye-enable-global -popeye-entry=popeye_main_ethernet_ip4_igmp -popeye-output=dot:igmpip4.dot -popeye-output=bnf

8. quic: a general-purpose transport layer network protocol
    https://github.com/ngtcp2/ngtcp2
    benchmarks/quic.bc -popeye-entry=popeye_main_hd_long -popeye-output=dot:quic.dot -popeye-output=bnf
    benchmarks/quic.bc -popeye-entry=popeye_main_hd_short -popeye-output=dot:quic.dot -popeye-output=bnf

9. babel: a distance-vector routing protocol that is designed to be robust and efficient on both wireless mesh networks and wired networks
    https://github.com/FRRouting/frr/
    benchmarks/babel.bc -popeye-entry=popeye_main_babel -popeye-output=dot:babel.dot -popeye-output=bnf

10. isis: intermediate system to intermediate system protocol
    https://github.com/FRRouting/frr/
    benchmarks/isis.bc -popeye-entry=popeye_main_isis -popeye-output=dot:isis.dot -popeye-output=bnf

