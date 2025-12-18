"use client";
import React from "react";
import Overview from "./components/overview/overview";
import Electrical from "./components/electrical/electrical";
import Mechanical from "./components/mechanical/mechanical";
import Software from "./components/software/software";
import ProjectManagement from "./components/project-management/project-management";

const page = () => {
  return (
    <>
      <main className="main">
        <Overview />
        <Mechanical />
        <Electrical />
        <Software />
        <ProjectManagement />
      </main>
    </>
  );
};

export default page;
