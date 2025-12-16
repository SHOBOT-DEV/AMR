import { useState } from "react";
import { loginUser } from "services/authLogin";
import { storeAuthTokens } from "utils/auth";

export default function useLogin(onLogin) {
  const [error, setError] = useState("");

  const submit = async (username, password) => {
    const { ok, data } = await loginUser(username, password);

    if (ok && data.success) {
      storeAuthTokens({ token: data.token, refreshToken: data.refreshToken });
      return onLogin();
    }

    setError(data.message || "Login failed");
  };

  return { error, submit };
}
