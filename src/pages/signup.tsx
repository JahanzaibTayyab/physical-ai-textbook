/**
 * Signup page with background questions.
 */

import React, { useState } from "react";
import Layout from "@theme/Layout";
import styles from "./auth.module.css";

export default function SignupPage(): React.JSX.Element {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [softwareBackground, setSoftwareBackground] = useState("");
  const [hardwareBackground, setHardwareBackground] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);
  const [success, setSuccess] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setLoading(true);

    try {
      const response = await fetch("http://localhost:8000/api/auth/signup", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          email,
          password,
          software_background: softwareBackground,
          hardware_background: hardwareBackground,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || "Signup failed");
      }

      const data = await response.json();
      // Store session token
      localStorage.setItem("session_token", data.session_token);
      localStorage.setItem("user_id", data.user_id);
      setSuccess(true);
      
      // Redirect to home after 2 seconds
      setTimeout(() => {
        window.location.href = "/";
      }, 2000);
    } catch (err) {
      setError(err instanceof Error ? err.message : "An error occurred");
    } finally {
      setLoading(false);
    }
  };

  if (success) {
    return (
      <Layout title="Sign Up">
        <div className={styles.container}>
          <div className={styles.successMessage}>
            <h2>Account Created Successfully!</h2>
            <p>Redirecting to homepage...</p>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign Up">
      <div className={styles.container}>
        <div className={styles.authCard}>
          <h1>Create Account</h1>
          <p className={styles.subtitle}>
            Sign up to access personalized content and translations
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
                minLength={8}
                disabled={loading}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="software-background">
                Software Background (Optional)
              </label>
              <textarea
                id="software-background"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value)}
                placeholder="e.g., Python developer, ROS 2 experience, web development..."
                rows={3}
                disabled={loading}
              />
              <small>
                Help us personalize content based on your software experience
              </small>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardware-background">
                Hardware Background (Optional)
              </label>
              <textarea
                id="hardware-background"
                value={hardwareBackground}
                onChange={(e) => setHardwareBackground(e.target.value)}
                placeholder="e.g., Robotics hardware, embedded systems, sensors..."
                rows={3}
                disabled={loading}
              />
              <small>
                Help us personalize content based on your hardware experience
              </small>
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? "Creating Account..." : "Sign Up"}
            </button>
          </form>

          <p className={styles.loginLink}>
            Already have an account? <a href="/signin">Sign in</a>
          </p>
        </div>
      </div>
    </Layout>
  );
}

