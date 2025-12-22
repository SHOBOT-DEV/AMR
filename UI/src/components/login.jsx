// import { useState } from "react";
// import "./login.css";
// import LeftImage from "../assets/Shobot_Img.png";
// import IITLogo from "../assets/IIT_Logo.png";

// const API_BASE = process.env.REACT_APP_API_BASE || "http://localhost:5000";

// const Login = ({ onLogin, onSignUpClick }) => {
//   const [username, setUsername] = useState("");
//   const [password, setPassword] = useState("");
//   const [error, setError] = useState("");

//   const handleSubmit = async (e) => {
//     e.preventDefault();
//     try {
//       const res = await fetch(`${API_BASE}/api/login`, {
//         method: "POST",
//         headers: { "Content-Type": "application/json" },
//         body: JSON.stringify({ username, password }),
//       });

//       // network-level ok but server error handling below
//       const data = await res.json().catch(() => ({}));

//       if (res.ok && data.success) {
//         // save token for future requests
//         localStorage.setItem("token", data.token);
//         // optionally save user info
//         localStorage.setItem("user", JSON.stringify(data.user));
//         setError("");
//         onLogin(); // proceed in app (navigate to main)
//         return;
//       }

//       // show server message or generic
//       setError(data.message || `Login failed (status ${res.status})`);
//     } catch (err) {
//       // Network error (server not running, CORS, etc.)
//       console.error("Login fetch error:", err);
//       setError(
//         `Network error: ${err.message}. Make sure Flask server is running at ${API_BASE} and CORS/proxy is configured.`
//       );
//     }
//   };

//   return (
//     <div className="login-container">
//       <div className="auth">
//         <div className="auth-inner-box">
//           <div className="auth-box">
//             <img src={IITLogo} alt="IIT logo" className="auth-logo" />
//             <h1>Login</h1>

//             {error && <p style={{ color: "red" }}>{error}</p>}

//             <form onSubmit={handleSubmit}>
//               <input
//                 type="text"
//                 placeholder="Username"
//                 value={username}
//                 onChange={(e) => setUsername(e.target.value)}
//                 required
//               />
//               <input
//                 type="password"
//                 placeholder="Password"
//                 value={password}
//                 onChange={(e) => setPassword(e.target.value)}
//                 required
//               />
//               <button type="submit">Login</button>
//             </form>

//             {/* ...existing code... */}
//           </div>
//         </div>
//       </div>
//     </div>
//   );
// };

// export default Login;