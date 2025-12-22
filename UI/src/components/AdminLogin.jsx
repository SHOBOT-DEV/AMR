// import { useState } from "react";
// import { useNavigate } from "react-router-dom";
// import "./login.css";

// const API_BASE = process.env.REACT_APP_API_BASE || "http://localhost:5000";

// const AdminLogin = () => {
//   const [username, setUsername] = useState("");
//   const [password, setPassword] = useState("");
//   const navigate = useNavigate();
//   const [error, setError] = useState("");

//   const handleSubmit = async (e) => {
//     e.preventDefault();
//     try {
//       const res = await fetch(`${API_BASE}/api/login`, {
//         method: "POST",
//         headers: { "Content-Type": "application/json" },
//         body: JSON.stringify({ username, password }),
//       });
//       const data = await res.json().catch(() => ({}));
//       if (res.ok && data.success) {
//         if (data.user && data.user.role === "admin") {
//           localStorage.setItem("token", data.token);
//           localStorage.setItem("user", JSON.stringify(data.user));
//           navigate("/admin/dashboard");
//         } else {
//           setError("Account is not an admin.");
//         }
//       } else {
//         setError(data.message || `Login failed (status ${res.status})`);
//       }
//     } catch (err) {
//       console.error("Admin login fetch error:", err);
//       setError(
//         `Network error: ${err.message}. Make sure Flask is running at ${API_BASE}`
//       );
//     }
//   };

//   return (
//     <div className="login-container">
//       <div className="login-image" style={{ flex: 1 }} />
//       <div className="auth" style={{ flex: 1, display: "flex", justifyContent: "flex-end", alignItems: "center", paddingRight: "6rem" }}>
//         <div className="auth-box" style={{ width: "80%", maxWidth: "420px" }}>
//           <h1>Admin Login</h1>

//           {error && <p style={{ color: "red" }}>{error}</p>}

//           <form onSubmit={handleSubmit}>
//             <input
//               type="text"
//               placeholder="Username"
//               value={username}
//               onChange={(e) => setUsername(e.target.value)}
//               required
//             />
//             <input
//               type="password"
//               placeholder="Password"
//               value={password}
//               onChange={(e) => setPassword(e.target.value)}
//               required
//             />
//             <button type="submit">Login</button>
//           </form>
//         </div>
//       </div>
//     </div>
//   );
// };

// export default AdminLogin;