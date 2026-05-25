import nextra from 'nextra';

const withNextra = nextra({
  theme: 'nextra-theme-docs',
  themeConfig: './theme.config.tsx',
  defaultShowCopyCode: true,
});

export default withNextra({
  output: 'export',
  images: { unoptimized: true },
  trailingSlash: true,
  // Set basePath to the repo name when deploying to https://<user>.github.io/<repo>/
  // Override at build time via NEXT_PUBLIC_BASE_PATH if you use a custom domain.
  basePath: process.env.NEXT_PUBLIC_BASE_PATH ?? '',
  assetPrefix: process.env.NEXT_PUBLIC_BASE_PATH ?? '',
});
