# CUDA compiler and TensorRT headers, prepared independently of detector source.
ARG DS_IMAGE=nvcr.io/nvidia/deepstream:7.1-samples-multiarch
FROM ${DS_IMAGE} AS dsyolo

ARG DS_VERSION
ENV DEBIAN_FRONTEND=noninteractive

# The TensorRT this machine needs, when the one in the base is not it.
#
# Two cases. A GPU newer than Hopper: DeepStream 7.1 ships TensorRT 10.3,
# which stops at Hopper. On a newer card it parses the ONNX, reports
# `Unsupported SM`, builds no engine and takes the process down, while the
# streams keep decoding. A newer TensorRT for jammy from the CUDA repository,
# which the x86 base already lists, fixes it. A Jetson: the generic arm64 base
# carries 10.3.0.26 and the host runs 10.3.0.30. An engine loads only under
# the build that made it, and 10.3.0.26 cannot build one on the Orin at all
# (a 5 MB tactic limit). The host's own packages fix it. They come from the
# JetPack repository, which TRT_APT_SOURCE names and this block adds.
# TRT_APT_PUBLIC_KEY is the URL of the key that signs that repository.
#
# TRT_CUDA names the CUDA runtime the x86 upgrade links. A Jetson leaves it
# empty, because its base already carries CUDA 12.6.
#
# Empty means the base's own TensorRT is right for this machine, which is the
# case for every x86 GPU up to Hopper. scripts/ds-select.sh decides all four.
ARG TRT_VERSION=
ARG TRT_CUDA=
ARG TRT_APT_SOURCE=
ARG TRT_APT_PUBLIC_KEY=
RUN if [ -n "${TRT_VERSION}" ]; then \
        set -eux; \
        apt-get update; \
        apt-get install -y --no-install-recommends ca-certificates curl; \
        if [ -n "${TRT_APT_SOURCE}" ]; then \
            curl -fsSL "${TRT_APT_PUBLIC_KEY}" -o /etc/apt/trusted.gpg.d/tensorrt-host.asc; \
            echo "${TRT_APT_SOURCE}" > /etc/apt/sources.list.d/tensorrt-host.list; \
            apt-get update; \
        fi; \
        cuda=""; \
        if [ -n "${TRT_CUDA}" ]; then cuda="cuda-cudart-${TRT_CUDA} libcublas-${TRT_CUDA}"; fi; \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer10=${TRT_VERSION}" \
            "libnvinfer-plugin10=${TRT_VERSION}" \
            "libnvonnxparsers10=${TRT_VERSION}" \
            "libnvinfer-dispatch10=${TRT_VERSION}" \
            "libnvinfer-lean10=${TRT_VERSION}" \
            "libnvinfer-vc-plugin10=${TRT_VERSION}" \
            "libnvinfer-bin=${TRT_VERSION}" \
            ${cuda}; \
        rm -rf /var/lib/apt/lists/*; \
        echo "TensorRT is now $(dpkg -l libnvinfer10 | awk '/^ii/ {print $3}')"; \
    fi

ARG YOLO_CUDA_VERSION=12.6
RUN set -eux; \
    CUDA_VER="${YOLO_CUDA_VERSION}"; \
    apt-get update; \
    apt-get install -y --no-install-recommends build-essential; \
    if [ ! -x "/usr/local/cuda-${CUDA_VER}/bin/nvcc" ]; then \
        apt-get install -y --no-install-recommends \
            "cuda-nvcc-$(echo "${CUDA_VER}" | tr . -)" \
            "cuda-cudart-dev-$(echo "${CUDA_VER}" | tr . -)" || true; \
    fi; \
    if [ ! -x "/usr/local/cuda-${CUDA_VER}/bin/nvcc" ]; then \
        found=$(ls -d /usr/local/cuda-*/bin/nvcc 2>/dev/null | sort -V | tail -1); \
        [ -n "${found}" ] || { echo "No CUDA compiler here and none installable. The parser cannot be built." >&2; exit 1; }; \
        CUDA_VER=$(echo "${found}" | sed "s|.*/cuda-||; s|/bin/nvcc||"); \
        echo "Falling back to the CUDA this base already carries: ${CUDA_VER}."; \
    fi; \
    if [ -n "${TRT_VERSION}" ]; then \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer-headers-dev=${TRT_VERSION}" \
            "libnvinfer-dev=${TRT_VERSION}" \
            "libnvinfer-headers-plugin-dev=${TRT_VERSION}" \
            "libnvinfer-plugin-dev=${TRT_VERSION}" \
            "libnvonnxparsers-dev=${TRT_VERSION}"; \
    fi; \
    for lib in /usr/local/cuda-*/lib64; do \
        [ -d "$lib" ] || continue; \
        for versioned in "$lib"/lib*.so.[0-9]*; do \
            plain="${versioned%%.so.*}.so"; \
            [ -e "$plain" ] || ln -s "$(basename "$versioned")" "$plain"; \
        done; \
    done; \
    mkdir -p /opt/ds-yolo; \
    printf '%s\n' "${YOLO_CUDA_VERSION}" > /opt/ds-yolo/cuda-requested; \
    printf '%s\n' "${CUDA_VER}" > /opt/ds-yolo/cuda-version; \
    rm -rf /var/lib/apt/lists/*
