import React from "react";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faChair } from "@fortawesome/free-solid-svg-icons";
import "./NavBar.css";

export default function NavBar() {
  return (
    <header className="navbar">
      <a href="#" className="navbar-icon">
        <FontAwesomeIcon icon={faChair} size=".5x" />
      </a>

      {/* Right: Navigation links */}
      <nav className="navbar-links">
        <a href="#overview">Overview</a>
        <a href="#mechanical">Mechanical</a>
        <a href="#electrical">Electrical</a>
        <a href="#software">Software</a>
        <a href="#project-management">Project Management</a>
      </nav>
    </header>
  );
}
