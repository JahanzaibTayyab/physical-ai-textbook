import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "Bridging the gap between the digital brain and the physical body",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://your-username.github.io", // Update with your GitHub username
  // Set the /<baseUrl>/ pathname under which your site is served
  // For local development, use "/". For GitHub pages, use "/<projectName>/"
  baseUrl: "/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "your-username", // Update with your GitHub org/user name.
  projectName: "physical-ai-textbook", // Usually your repo name.

  onBrokenLinks: "throw",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en", // English as default language
    locales: ["en", "ur"], // English first, then Urdu
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          routeBasePath: "/docs", // Serve docs at /docs
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/your-username/physical-ai-textbook/tree/main/", // Update with your repo
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ["rss", "atom"],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/",
          // Useful options to enforce blogging best practices
          onInlineTags: "warn",
          onInlineAuthors: "warn",
          onUntruncatedBlogPosts: "warn",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],
  clientModules: ["./src/components/Chatbot/client-module.tsx"],

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "courseSidebar",
          position: "left",
          label: "Course",
        },
        {
          type: "localeDropdown",
          position: "right",
        },
        {
          href: "https://github.com/your-username/physical-ai-textbook", // Update with your repo
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Course Modules",
          items: [
            {
              label: "Module 1: ROS 2",
              to: "/module-1-ros2/intro",
            },
            {
              label: "Module 2: Gazebo & Unity",
              to: "/module-2-simulation/intro",
            },
            {
              label: "Module 3: NVIDIA Isaac",
              to: "/module-3-isaac/intro",
            },
            {
              label: "Module 4: VLA",
              to: "/module-4-vla/intro",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "Panaversity",
              href: "https://panaversity.org",
            },
            {
              label: "Spec-Kit Plus",
              href: "https://github.com/panaversity/spec-kit-plus/",
            },
            {
              label: "Claude Code",
              href: "https://www.claude.com/product/claude-code",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/your-username/physical-ai-textbook", // Update with your repo
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
