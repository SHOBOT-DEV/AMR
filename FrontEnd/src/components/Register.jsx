
import "./Register.css";
import LeftImage from "../assets/Shobot_Img.png";
import IITLogo from "../assets/IIT_Logo.png";

const Register = ({ onRegister, onLoginClick }) => {
  const handleSubmit = (e) => {
    e.preventDefault();
    onRegister();
  };

  return (
    <div className="register-container" style={{ display: "flex", height: "100vh", width: "100vw" }}>
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

          <form onSubmit={handleSubmit}>
            <input type="text" placeholder="Username" required />
            <input type="email" placeholder="Email" required />
            <input type="password" placeholder="Password" required />
            <input type="text" placeholder="Company" required />
            <select required>
              <option value="">Select AMR Type</option>
              <option value="Type A">AMR 100</option>
              <option value="Type B">AMR 200</option>
              <option value="Type C">AMR 300</option>
              <option value="Type C">AMR 500</option>



            </select>
            <button type="submit">Register</button>
          </form>

          <p className="login-message">
            Already have an account?{" "}
            <span onClick={onLoginClick} style={{ cursor: "pointer", color: "#0056d2" }}>
              Login
            </span>
          </p>
        </div>
      </div>
    </div>
  );
};

export default Register;
