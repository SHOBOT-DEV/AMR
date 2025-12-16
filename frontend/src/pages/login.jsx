import IITLogo from "assets/IIT_Logo.png";
import LeftImage from "assets/Shobot_Img.png";
import useLogin from "hooks/useLogin";
import LoginForm from "components/LoginForm";

export default function Login({ onLogin, onSignUpClick }) {
  const { error, submit } = useLogin(onLogin);

  return (
    <div className="flex h-screen w-full">
      <div
        className="hidden lg:block flex-1 bg-cover bg-center"
        style={{ backgroundImage: `url(${LeftImage})` }}
      />

      <div className="flex flex-1 items-center justify-center bg-gradient-to-br from-cyan-200 to-blue-400 p-6">
        <div className="bg-white p-10 rounded-xl shadow-lg max-w-md w-full">
          <img src={IITLogo} className="w-20 mx-auto mb-6" />
          <h1 className="text-3xl text-center mb-6 font-semibold">Login</h1>

          <LoginForm
            error={error}
            onSubmit={submit}
            onSignUpClick={onSignUpClick}
          />
        </div>
      </div>
    </div>
  );
}
