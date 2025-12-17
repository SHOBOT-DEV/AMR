// import { useState } from "react";

// export default function LoginForm({ error, onSubmit, onSignUpClick }) {
//   const [form, setForm] = useState({ user: "", pass: "" });

//   const update = (e) =>
//     setForm({ ...form, [e.target.name]: e.target.value });

//   const submit = (e) => {
//     e.preventDefault();
//     onSubmit(form.user.trim(), form.pass.trim());
//   };

//   return (
//     <form onSubmit={submit} className="space-y-4">
//       {error && <p className="text-red-600 text-center">{error}</p>}

//       <input
//         name="user"
//         placeholder="Username"
//         className="input"
//         value={form.user}
//         onChange={update}
//         required
//       />

//       <input
//         name="pass"
//         placeholder="Password"
//         type="password"
//         className="input"
//         value={form.pass}
//         onChange={update}
//         required
//       />

//       <button className="btn w-full">Login</button>

//       <p className="text-blue-600 text-center cursor-pointer" onClick={onSignUpClick}>
//         Sign up
//       </p>
//     </form>
//   );
// }
