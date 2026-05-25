import React from 'react';
import { DocsThemeConfig } from 'nextra-theme-docs';

const config: DocsThemeConfig = {
  logo: (
    <span
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: 10,
        fontWeight: 600,
        letterSpacing: '-0.01em',
      }}
    >
      <span
        aria-hidden="true"
        style={{
          display: 'inline-block',
          width: 22,
          height: 22,
          borderRadius: 6,
          background:
            'linear-gradient(135deg, #008060 0%, #00A074 50%, #95BF47 100%)',
          boxShadow: '0 1px 0 rgba(0,0,0,.04), inset 0 1px 0 rgba(255,255,255,.18)',
        }}
      />
      <span>Magalhães</span>
      <span
        style={{
          marginLeft: 2,
          color: 'var(--shopify-muted)',
          fontWeight: 500,
          fontSize: '0.9em',
        }}
      >
        Flight Computer
      </span>
    </span>
  ),
  project: {
    link: 'https://github.com/RED-PT/R-D_Software_Magalhaes_V2',
  },
  docsRepositoryBase:
    'https://github.com/RED-PT/R-D_Software_Magalhaes_V2/blob/main',
  useNextSeoProps() {
    return { titleTemplate: '%s — Magalhães' };
  },
  head: (
    <>
      <meta name="viewport" content="width=device-width, initial-scale=1.0" />
      <meta
        name="description"
        content="Documentation for the Magalhães TVC rocket flight computer."
      />
      <meta property="og:title" content="Magalhães Flight Computer" />
      <meta
        property="og:description"
        content="A TVC rocket flight computer — documentation, references, and the story of how it works."
      />
      <link rel="preconnect" href="https://rsms.me/" />
      <link rel="stylesheet" href="https://rsms.me/inter/inter.css" />
    </>
  ),
  // Shopify Polaris green ≈ HSL(165, 100, 25). Nextra mixes this with lightness
  // for text/background tints; we override the deepest tones in globals.css.
  primaryHue: 165,
  primarySaturation: 80,
  sidebar: {
    defaultMenuCollapseLevel: 1,
    toggleButton: true,
  },
  toc: {
    backToTop: true,
    float: true,
    title: 'On this page',
  },
  editLink: { text: 'Edit this page on GitHub' },
  feedback: { content: null },
  footer: {
    text: (
      <span style={{ fontSize: '0.85em', opacity: 0.8 }}>
        © {new Date().getFullYear()} R&amp;D Software · AeroTéc — Magalhães Flight Computer
      </span>
    ),
  },
  banner: undefined,
  search: { placeholder: 'Search documentation…' },
  darkMode: true,
  nextThemes: { defaultTheme: 'system' },
  navigation: { prev: true, next: true },
};

export default config;
