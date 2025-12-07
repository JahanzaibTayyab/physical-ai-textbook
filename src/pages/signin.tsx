/**
 * Signin page.
 */

import React, { useState } from "react";
import Layout from "@theme/Layout";
import styles from "./auth.module.css";

export default function SigninPage(): React.JSX.Element {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    try {
      const response = await fetch("http://localhost:8000/api/auth/signin", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          email,
          password,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || "Signin failed");
      }

      const data = await response.json();
      // Store session token
      localStorage.setItem("session_token", data.session_token);
      localStorage.setItem("user_id", data.user_id);
      
      // Redirect to home
      window.location.href = "/";
    } catch (err) {
      setError(err instanceof Error ? err.message : "An error occurred");
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign In">
      <div className={styles.container}>
        <div className={styles.authCard}>
          <h1>Sign In</h1>
          <p className={styles.subtitle}>
            Sign in to access personalized content and translations
          </p>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSubmit} className={styles.form}>
            <div className={styles.formGroup}>
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                disabled={loading}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                disabled={loading}
              />
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? "Signing In..." : "Sign In"}
            </button>
          </form>

          <p className={styles.loginLink}>
            Don't have an account? <a href="/signup">Sign up</a>
          </p>
        </div>
      </div>
    </Layout>
  );
}

