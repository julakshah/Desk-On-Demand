import withMDX from "@next/mdx";

/** @type {import('next').NextConfig} */
const nextConfig = {
  reactCompiler: true,
  pageExtensions: ["js", "jsx", "ts", "tsx", "mdx"], // include MDX files
  experimental: {
    appDir: true, // App Router support
  },
  output: "export", // enable static export for GitHub Pages
};

// Wrap the config with MDX support
export default withMDX({
  extension: /\.mdx?$/,
  options: {
    remarkPlugins: [],
    rehypePlugins: [],
  },
})(nextConfig);
