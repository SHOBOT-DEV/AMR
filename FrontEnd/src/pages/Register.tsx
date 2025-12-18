import { useState, type FormEvent } from "react";
import LeftImage from "../assets/Shobot_Img.png";
import IITLogo from "../assets/IIT_Logo.png";

interface RegisterProps {
  onRegister: () => void;
  onLoginClick: () => void;
}

const Register = ({ onRegister, onLoginClick }: RegisterProps) => {
  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [company, setCompany] = useState("");
  const [amrType, setAmrType] = useState("");
  const [error, setError] = useState("");
  const [success, setSuccess] = useState("");

  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
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
          amrType,
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        setSuccess("Registration successful! Redirecting to login...");
        setTimeout(onRegister, 1500);
      } else {
        setError(data.message || "Registration failed. Please try again.");
      }
    } catch {
      setError("Network error. Make sure Flask server is running on port 5000.");
    }
  };

  return (
    <div className="flex h-screen w-screen font-[Poppins]">
      {/* Left image */}
      <div
        className="hidden md:block w-1/2 bg-cover bg-center"
        style={{ backgroundImage: `url(${LeftImage})` }}
      />

      {/* Right */}
      <div className="flex w-full md:w-1/2 items-center justify-end pr-24">
        <div className="w-full max-w-md rounded-xl bg-white p-12 shadow-xl">
          <img
            src={IITLogo}
            alt="IIT logo"
            className="mx-auto mb-4 w-20"
          />

          <h1 className="mb-6 text-center text-2xl font-semibold text-gray-800">
            Register
          </h1>

          {error && <p className="mb-3 text-red-500">{error}</p>}
          {success && <p className="mb-3 text-green-600">{success}</p>}

          <form onSubmit={handleSubmit} className="space-y-4">
            <input
              className="w-full rounded-md border border-gray-300 p-3"
              type="text"
              placeholder="Username"
              value={username}
              onChange={(e) => setUsername(e.target.value)}
              required
            />

            <input
              className="w-full rounded-md border border-gray-300 p-3"
              type="email"
              placeholder="Email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />

            <input
              className="w-full rounded-md border border-gray-300 p-3"
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />

            <input
              className="w-full rounded-md border border-gray-300 p-3"
              type="text"
              placeholder="Company"
              value={company}
              onChange={(e) => setCompany(e.target.value)}
              required
            />

            <select
              className="w-full rounded-md border border-gray-300 p-3"
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

            <button
              type="submit"
              className="w-full rounded-md bg-blue-600 py-3 text-white transition hover:bg-blue-800"
            >
              Register
            </button>
          </form>

          <p className="mt-4 text-center text-sm text-gray-700">
            Already have an account?{" "}
            <span
              onClick={onLoginClick}
              className="cursor-pointer font-semibold text-blue-600 hover:underline"
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
