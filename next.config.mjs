import withMDX from "@next/mdx";

/** @type {import('next').NextConfig} */
const nextConfig = {
  reactCompiler: true,
  pageExtensions: ["js", "jsx", "ts", "tsx", "mdx"], // include MDX files
  experimental: {
    appDir: true, // ensures App Router support
  },
};

// Wrap the config with MDX support
export default withMDX({
  extension: /\.mdx?$/,
  options: {
    remarkPlugins: [],
    rehypePlugins: [],
  },
})(nextConfig);
