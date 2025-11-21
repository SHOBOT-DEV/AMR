import { useState } from "react";
import { useNavigate } from "react-router-dom";
import "./Admin.css"; // Reuse your login styling
import LeftImage from "../assets/Shobot_WBg.png"; // Same left-side image
import IITLogo from "../assets/IIT_Logo.png"; // Logo at the top

const AdminLogin = () => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const navigate = useNavigate();

  const handleSubmit = (e) => {
    e.preventDefault();

    if (username === "admin" && password === "admin123") {
      setError(""); 
      navigate("/admin/dashboard"); // Redirect to admin dashboard
    } else {
      setError("Invalid credentials, please try again.");
    }
  };

  return (
    <div className="login-container">
      {/* Left half: background image */}
      <div
        className="login-image"
        style={{ backgroundImage: `url(${LeftImage})` }}
      />

      {/* Right half: auth box */}
      <div className="auth">
        <div className="auth-box">
          <img src={IITLogo} alt="IIT Logo" className="auth-logo" />
          <h1>Admin Login</h1>

          {error && <p style={{ color: "red" }}>{error}</p>}

          <form onSubmit={handleSubmit}>
            <input
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />
            <input
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
            <button type="submit">Admin Login</button>
          </form>
        </div>
      </div>
    </div>
  );
};

export default AdminLogin;
