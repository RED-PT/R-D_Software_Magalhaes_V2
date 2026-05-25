import React from 'react';

/**
 * Inline link to a function in the Doxygen API reference.
 *
 *   <FuncRef name="ASM330LHHX_Init" file="ASM330LHHX.h" />
 *
 * Renders a small pill that links to /api/<file slug>.html. Doxygen
 * slugs file names like `ASM330LHHX_8h.html`, so the consumer just
 * passes the bare filename.
 */
export function FuncRef({
  name,
  file,
  href,
}: {
  name: string;
  file?: string;
  href?: string;
}) {
  // If consumer provides an explicit href, use it. Otherwise build a search-style link.
  const to =
    href ??
    (file
      ? `/api/${doxygenSlug(file)}.html`
      : `/api/search.html?query=${encodeURIComponent(name)}`);

  return (
    <a className="magalhaes-funcref" href={to}>
      <code>{name}()</code>
      <span aria-hidden="true">↗</span>
    </a>
  );
}

/** Convert "ASM330LHHX.h" → "_a_s_m330_l_h_h_x_8h" (Doxygen file-anchor scheme). */
function doxygenSlug(filename: string): string {
  // Doxygen lowercases, prefixes underscore for any uppercase, replaces "." with "_8".
  const dotIdx = filename.lastIndexOf('.');
  const base = dotIdx >= 0 ? filename.slice(0, dotIdx) : filename;
  const ext = dotIdx >= 0 ? filename.slice(dotIdx + 1) : '';
  let slug = '';
  for (const ch of base) {
    if (ch >= 'A' && ch <= 'Z') {
      slug += '_' + ch.toLowerCase();
    } else {
      slug += ch;
    }
  }
  return ext ? `${slug}_8${ext}` : slug;
}

export default FuncRef;
