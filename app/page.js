"use client";
import React from "react";
import Overview from "./components/overview/overview";
import Mechanical from "./components/mechanical/mechanical";
import Electrical from "./components/electrical/electrical";
import Software from "./components/software/software";
import ProjectManagement from "./components/project-management/project-management";
import FrameUpdate from "./components/frame-updates/frames";

export default function Page() {
  return (
    <>
      {/* Hero section */}
      <section className="hero">
        <div className="hero-content">
          <h1>Desk On Demand</h1>
        </div>
      </section>

      {/* Other sections */}
      <section id="overview" className="card">
        <Overview />
      </section>
      <section id="mechanical" className="card">
        <Mechanical />
      </section>
      <section id="electrical" className="card">
        <Electrical />
      </section>
      <section id="software" className="card">
        <Software />
      </section>
      <section id="project-management" className="card">
        <ProjectManagement />
      </section>
      <section id="frame-update" className="card">
        <FrameUpdate />
      </section>
    </>
  );
}
