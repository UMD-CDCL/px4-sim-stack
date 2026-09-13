#!/usr/bin/env bash
# Publish a complete parser library onto a shared model volume.
set -euo pipefail
PARSER_SRC=${1:?parser source path required}
PARSER_DIR=${2:?model directory required}
DS_RELEASE=${3:?DeepStream release required}
PARSER_DEST="${PARSER_DIR}/libnvdsinfer_custom_impl_Yolo.so"
PARSER_STAMP="${PARSER_DIR}/.parser-deepstream"
if [ -f "${PARSER_SRC}" ] && [ -w "${PARSER_DIR}" ]; then
	want="${DS_RELEASE:-unknown}:$(sha256sum "${PARSER_SRC}" | cut -d' ' -f1)"
	have=$(cat "${PARSER_STAMP}" 2>/dev/null || echo none)
	if [ ! -e "${PARSER_DEST}" ] || [ "${have}" != "${want}" ]; then
		# Containers may start together against the same model volume. Rename
		# a complete temporary file so no reader sees a half-written library.
		parser_tmp=$(mktemp "${PARSER_DIR}/.parser.XXXXXX")
		cp "${PARSER_SRC}" "$parser_tmp"
		chmod 0644 "$parser_tmp"
		mv -f "$parser_tmp" "${PARSER_DEST}"
		stamp_tmp=$(mktemp "${PARSER_DIR}/.parser-stamp.XXXXXX")
		printf '%s\n' "${want}" > "$stamp_tmp"
		mv -f "$stamp_tmp" "${PARSER_STAMP}"
		echo "parser: ${PARSER_DEST} is the DeepStream ${want} build (was ${have})"
	fi
elif [ ! -e "${PARSER_DEST}" ]; then
	echo "parser: no ${PARSER_DEST} and none to install. ds_node will try to" >&2
	echo "        compile one, which needs a CUDA compiler this image has not got." >&2
fi
