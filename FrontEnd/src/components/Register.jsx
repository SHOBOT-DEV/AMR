import { useState } from "react";
import "./Register.css";
import LeftImage from "../assets/Shobot_Img.png";
import IITLogo from "../assets/IIT_Logo.png";

const Register = ({ onRegister, onLoginClick }) => {
  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [company, setCompany] = useState("");
  const [amrType, setAmrType] = useState("");
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");
    setSuccess("");

    try {
      const response = await fetch("http://127.0.0.1:5000/api/register", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          username: username.trim(),
          email: email.trim(),
          password: password.trim(),
          company: company.trim(),
          amrType: amrType,
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        setSuccess("Registration successful! Redirecting to login...");
        setTimeout(() => {
          onRegister();
        }, 1500);
      } else {
        setError(data.message || "Registration failed. Please try again.");
      }
    } catch (err) {
      console.error(err);
      setError(
        "Network error. Make sure Flask server is running on port 5000.",
      );
    }
  };

  return (
    <div
      className="register-container"
      style={{ display: "flex", height: "100vh", width: "100vw" }}
    >
      {/* Left half */}
      <div
        className="register-image"
        style={{
          backgroundImage: `url(${LeftImage})`,
          width: "50%",
          backgroundSize: "cover",
          backgroundPosition: "center",
        }}
      />

      {/* Right half */}
      <div
        className="auth"
        style={{
          width: "50%",
          display: "flex",
          justifyContent: "flex-end",
          alignItems: "center",
          paddingRight: "6rem",
        }}
      >
        <div className="auth-box" style={{ width: "80%", maxWidth: "420px" }}>
          <img src={IITLogo} alt="IIT logo" className="auth-logo" />
          <h1>Register</h1>

          {error && <p style={{ color: "red" }}>{error}</p>}
          {success && <p style={{ color: "green" }}>{success}</p>}

          <form onSubmit={handleSubmit}>
            <input
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />
            <input
              type="email"
              placeholder="Email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
            <input
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
            <input
              type="text"
              placeholder="Company"
              value={company}
              onChange={(e) => setCompany(e.target.value)}
              required
            />
            <select
              value={amrType}
              onChange={(e) => setAmrType(e.target.value)}
              required
            >
              <option value="">Select AMR Type</option>
              <option value="AMR 100">AMR 100</option>
              <option value="AMR 200">AMR 200</option>
              <option value="AMR 300">AMR 300</option>
              <option value="AMR 500">AMR 500</option>
            </select>
            <button type="submit">Register</button>
          </form>

          <p className="login-message">
            Already have an account?{" "}
            <span
              onClick={onLoginClick}
              style={{ cursor: "pointer", color: "#0056d2" }}
            >
              Login
            </span>
          </p>
        </div>
      </div>
    </div>
  );
};

export default Register;
