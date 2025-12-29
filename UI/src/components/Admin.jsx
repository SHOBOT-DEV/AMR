// import { useState, useEffect } from "react";
// import { useNavigate } from "react-router-dom";
// import "./Admin.css";
// import IITLogo from "../assets/IIT_Logo.png";

// const Admin = () => {
//   const navigate = useNavigate();
//   const [users, setUsers] = useState([]); // removed sample users
//   const [loading, setLoading] = useState(true);
//   const [error, setError] = useState("");

//   useEffect(() => {
//     const token = localStorage.getItem("token");
//     if (!token) {
//       navigate("/admin"); // no token -> admin login
//       return;
//     }

//     const fetchUsers = async () => {
//       try {
//         const res = await fetch("/api/admin/users", {
//           method: "GET",
//           headers: {
//             "Content-Type": "application/json",
//             Authorization: `Bearer ${token}`,
//           },
//         });

//         if (res.status === 401 || res.status === 403) {
//           // token missing/invalid or not admin
//           localStorage.removeItem("token");
//           localStorage.removeItem("user");
//           navigate("/admin");
//           return;
//         }

//         const data = await res.json();
//         setUsers(Array.isArray(data) ? data : []);
//         setLoading(false);
//       } catch (err) {
//         setError("Failed to fetch users");
//         setLoading(false);
//       }
//     };

//     fetchUsers();
//   }, [navigate]);

//   const handleLogout = () => {
//     localStorage.removeItem("token");
//     localStorage.removeItem("user");
//     navigate("/admin");
//   };

//   return (
//     <div className="admin-container">
//       {/* ...existing sidebar code... */}
//       <div className="admin-sidebar">
//         <div className="sidebar-header">
//           <img src={IITLogo} alt="IIT Logo" className="sidebar-logo" />
//           <h2>Admin Panel</h2>
//         </div>

//         {/* ...existing sidebar menu ... */}

//         <button className="logout-btn" onClick={handleLogout}>
//           Logout
//         </button>
//       </div>

//       {/* Main content area */}
//       <div className="admin-content">
//         <h1>Welcome, Admin 👋</h1>
//         <p>Manage users below. Use the approval column to review status.</p>

//         <div className="table-container">
//           {loading ? (
//             <div>Loading users...</div>
//           ) : error ? (
//             <div style={{ color: "red" }}>{error}</div>
//           ) : (
//             <table className="user-table">
//               <thead>
//                 <tr>
//                   <th>Username</th>
//                   <th>Email</th>
//                   <th>Role</th>
//                   <th>Approval</th>
//                 </tr>
//               </thead>
//               <tbody>
//                 {users.map((u, idx) => (
//                   <tr key={idx}>
//                     <td>{u.username}</td>
//                     <td>{u.email}</td>
//                     <td>{u.role}</td>
//                     <td>{u.approval}</td>
//                   </tr>
//                 ))}
//               </tbody>
//             </table>
//           )}
//         </div>
//       </div>
//     </div>
//   );
// };

// export default Admin;