#!/usr/bin/env bash
# Fetch Doxygen Awesome theme assets into ./doxygen-awesome/
# Run before `doxygen Doxyfile` if building docs locally.
# CI runs the same fetch in .github/workflows/docs.yml.

set -euo pipefail

VERSION="v2.3.4"
BASE="https://raw.githubusercontent.com/jothepro/doxygen-awesome-css/${VERSION}"
DEST="$(dirname "$0")/../doxygen-awesome"
mkdir -p "${DEST}"

files=(
  "doxygen-awesome.css"
  "doxygen-awesome-sidebar-only.css"
  "doxygen-awesome-sidebar-only-darkmode-toggle.css"
  "doxygen-awesome-darkmode-toggle.js"
  "doxygen-awesome-fragment-copy-button.js"
  "doxygen-awesome-paragraph-link.js"
  "doxygen-awesome-interactive-toc.js"
)

for f in "${files[@]}"; do
  echo "Fetching ${f}..."
  curl -fsSL "${BASE}/${f}" -o "${DEST}/${f}"
done

echo "Done. Theme assets in ${DEST}/"
