import { useState } from "react";
import "./login.css"; // you can reuse your login styles

const AdminLogin = ({ onAdminLogin, onBackClick }) => {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      const res = await fetch("/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ username, password }),
      });
      const data = await res.json();
      if (res.ok && data.success) {
        // ensure admin role
        if (data.user && data.user.role === "admin") {
          localStorage.setItem("token", data.token);
          localStorage.setItem("user", JSON.stringify(data.user));
          onAdminLogin(); // parent should navigate to admin dashboard
        } else {
          alert("Not an admin account");
        }
      } else {
        alert(data.message || "Login failed");
      }
    } catch (err) {
      alert("Network error");
    }
  };

  // Example function: fetch protected admin users (call from admin dashboard)
  // const fetchAdminUsers = async () => {
  //   const token = localStorage.getItem("token");
  //   const res = await fetch("/api/admin/users", {
  //     method: "GET",
  //     headers: {
  //       "Content-Type": "application/json",
  //       Authorization: `Bearer ${token}`,
  //     },
  //   });
  //   const users = await res.json();
  //   console.log(users);
  // };

  return (
    <div
      className="login-container"
      style={{ display: "flex", height: "100vh" }}
    >
      {/* ...existing code... */}
      <div
        className="auth"
        style={{
          flex: 1,
          display: "flex",
          justifyContent: "flex-end",
          alignItems: "center",
          paddingRight: "6rem",
        }}
      >
        <div className="auth-box" style={{ width: "80%", maxWidth: "420px" }}>
          <h1>Admin Login</h1>
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
          <p
            className="forgot-password"
            onClick={onBackClick}
            style={{ cursor: "pointer" }}
          >
            Back to User Login
          </p>
        </div>
      </div>
    </div>
  );
};

export default AdminLogin;
