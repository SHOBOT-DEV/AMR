import { useState } from "react";
import "./login.css";
import LeftImage from "../assets/Shobot_Img.png";
import IITLogo from "../assets/IIT_Logo.png";
import { storeAuthTokens, API_BASE } from "../utils/auth";

const Login = ({ onLogin, onSignUpClick }) => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const handleSubmit = async (e) => {
    e.preventDefault();

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
        setError("");
        storeAuthTokens({
          token: data.token,
          refreshToken: data.refreshToken,
        });
        onLogin();
      } else {
        setError(data.message || "Login failed. Please try again.");
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
      <div
        className="login-image"
        style={{ backgroundImage: `url(${LeftImage})` }}
      />

      <div className="auth">
        <div className="auth-box">
          <img src={IITLogo} alt="IIT logo" className="auth-logo" />
          <h1>Login</h1>
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
            <button type="submit">Login</button>
          </form>

          <span
            onClick={onSignUpClick}
            style={{
              cursor: "pointer",
              color: "#0056d2",
              marginTop: "20px",
              display: "block",
            }}
          >
            Sign up
          </span>
        </div>
      </div>
    </div>
  );
};

export default Login;
