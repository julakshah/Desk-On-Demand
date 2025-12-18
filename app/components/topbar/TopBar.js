// app/components/TopBar.js
import React from "react";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faChair } from "@fortawesome/free-solid-svg-icons";
import "./TopBar.css";

export default function TopBar() {
  return (
    <header className="topbar">
      <a href="/#overview" className="nav__link">
        <FontAwesomeIcon icon={faChair} />
      </a>

      <nav className="topbar-nav">
        <a href="#overview">Overview</a>
        <a href="#mechanical">Mechanical</a>
        <a href="#electrical">Electrical</a>
        <a href="#software">Software</a>
        <a href="#project-management">Project Management</a>
      </nav>
    </header>
  );
}
