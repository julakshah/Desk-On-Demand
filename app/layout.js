import "./globals.css";
import { Inconsolata } from "next/font/google";

const inconsolata = Inconsolata({
  subsets: ["latin"],
  weights: ["400", "700"],
});

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <head>
        <title>Desk On Demand</title>
      </head>
      <body>{children}</body>
    </html>
  );
}
