// import React from "react";
// import { useEffect, useRef } from "react";
import "./RightPanel.css";

import { useEffect, useRef } from "react";
import nipplejs from "nipplejs";

const joystickPanelStyle = {
  position: "fixed",
  top: "50%",
  right: 20,
  transform: "translateY(-50%)",
  width: 180,
  height: 180,
  backgroundColor: "#222",
  borderRadius: "20px",
  boxShadow: "0 0 15px rgba(0,0,0,0.7)",
  display: "flex",
  justifyContent: "center",
  alignItems: "center",
  zIndex: 1000,
};

function Joystick() {
  const joystickRef = useRef(null);

  useEffect(() => {
    const joystick = nipplejs.create({
      zone: joystickRef.current,
      mode: "static",
      position: { top: "50%", left: "50%" },
      color: "#888",
      size: 120,
    });

    joystick.on("move", (evt, data) => {
      // Handle move data
      console.log("Direction:", data.direction);
      console.log("Distance:", data.distance);
    });

    joystick.on("end", () => {
      console.log("Joystick released");
    });

    return () => {
      joystick.destroy();
    };
  }, []);

  return (
    <div style={joystickPanelStyle}>
      <div
        ref={joystickRef}
        className="joystick-zone"
        style={{ width: 150, height: 150 }}
      />
    </div>
  );
}

export default Joystick;
