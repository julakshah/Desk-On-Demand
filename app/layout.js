import "./globals.css";
import { Inconsolata } from "next/font/google";
import TopBar from "./components/topbar/TopBar.js";

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
      <body className={inconsolata.className}>
        <TopBar />
        <main className="container">{children}</main>
      </body>
    </html>
  );
}
