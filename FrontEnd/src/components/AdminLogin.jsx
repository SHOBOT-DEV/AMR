import { useState } from "react";
import { useNavigate } from "react-router-dom";
import "./login.css"; // Reuse your login styling
import LeftImage from "../assets/Shobot_WBg.png"; // Same left-side image
import IITLogo from "../assets/IIT_Logo.png"; // Logo at the top
import { storeAuthTokens, API_BASE } from "../utils/auth";

const AdminLogin = () => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const navigate = useNavigate();

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError("");

    try {
      const response = await fetch(`${API_BASE}/api/login`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          username: username.trim(),
          password: password.trim(),
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Check if user is admin
        if (data.user.role === "admin") {
          storeAuthTokens({
            token: data.token,
            refreshToken: data.refreshToken,
            isAdmin: true,
          });
          setError("");
          navigate("/admin/dashboard");
        } else {
          setError("Admin access required. This account is not an admin.");
        }
      } else {
        setError(data.message || "Invalid credentials, please try again.");
      }
    } catch (err) {
      console.error(err);
      setError(
        "Network error. Make sure Flask server is running on port 5000.",
      );
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
