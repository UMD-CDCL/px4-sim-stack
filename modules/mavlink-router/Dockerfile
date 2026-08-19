# The MAVLink contract: one router for each vehicle, configured from the same
# template a Chimera companion computer runs.
#
# One container for each vehicle, because every vehicle binds TCP 5760 and its
# own network namespace is the only thing that lets them all have it.

FROM debian:bookworm-slim AS build

ARG MAVLINK_ROUTER_REF=v4

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      git \
      meson \
      ninja-build \
      pkg-config \
      python3 \
    && rm -rf /var/lib/apt/lists/*

# systemdsystemunitdir defaults to "auto", and meson.build then calls
# dependency('systemd') to read the unit directory out of systemd.pc. Debian
# bookworm does not ship that .pc file in any package this image wants, so the
# configure step fails with "Dependency systemd not found". Giving the option a
# value skips the lookup. The unit file it installs is never copied out.
RUN git clone --recurse-submodules --depth 1 --branch "${MAVLINK_ROUTER_REF}" \
      https://github.com/mavlink-router/mavlink-router.git /src

# Upstream defect in the v4 tag: the TCP server is broken for IPv4 clients.
#
#   Mainloop::tcp_open()   creates the listener with AF_INET6, bound to [::].
#   TcpEndpoint::accept()  sets is_ipv6 = false, "TCP server is IPv4 only for
#                          now", and accepts into a sockaddr_in.
#   TcpEndpoint::write_msg() then calls sendto() with that IPv4 address on an
#                          IPv6 socket, and the kernel rejects the family
#                          mismatch with EINVAL.
#
# The symptom is a flood of "Error sending tcp packet (Invalid argument)" and a
# TCP client that connects, sends fine, and receives nothing. Receiving works,
# so the failure is silent from the client side.
#
# The listener is AF_INET6, so the accepted socket is too. One word fixes it.
RUN cd /src \
    && if grep -q 'is_ipv6 = false;.*TCP server is IPv4 only for now' src/endpoint.cpp; then \
         sed -i 's|is_ipv6 = false;.*TCP server is IPv4 only for now|is_ipv6 = true; // patched: the listener is AF_INET6, so accepted sockets are too|' \
           src/endpoint.cpp \
         && echo "patched TcpEndpoint::accept for IPv4-mapped clients"; \
       else \
         echo "NOTE: the IPv6 accept line is gone. Upstream may have fixed it. Check TCP before you trust it."; \
       fi

RUN meson setup /build /src --prefix=/usr --buildtype=release \
         -Dsystemdsystemunitdir=/lib/systemd/system \
    && ninja -C /build \
    && DESTDIR=/out ninja -C /build install

# ---------------------------------------------------------------------------

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      gettext-base \
      python3-minimal \
      iproute2 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /out/usr/bin/mavlink-routerd /usr/bin/mavlink-routerd

COPY keepalive.py /usr/local/bin/keepalive.py
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY main.conf.template /opt/mavlink-router/main.conf.template
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/keepalive.py

EXPOSE 5760/tcp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
