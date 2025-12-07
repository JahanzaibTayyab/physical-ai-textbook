import Heading from "@theme/Heading";
import type { ReactNode } from "react";
import clsx from "clsx";
import styles from "./styles.module.css";

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<"svg">>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: "ROS 2 Framework",
    Svg: require("@site/static/img/undraw_docusaurus_mountain.svg").default,
    description: (
      <>
        Master the Robotic Operating System 2 (ROS 2) - the industry standard
        for building intelligent robotic systems.
      </>
    ),
  },
  {
    title: "Digital Twin Simulation",
    Svg: require("@site/static/img/undraw_docusaurus_tree.svg").default,
    description: (
      <>
        Build realistic simulations with Gazebo and Unity. Create digital twins
        of your robots and test in virtual environments.
      </>
    ),
  },
  {
    title: "AI-Powered Robotics",
    Svg: require("@site/static/img/undraw_docusaurus_react.svg").default,
    description: (
      <>
        Integrate NVIDIA Isaac Sim, reinforcement learning, and
        Vision-Language-Action (VLA) models to create intelligent, autonomous
        robotic systems.
      </>
    ),
  },
];

function Feature({ title, Svg, description }: FeatureItem) {
  return (
    <div className={clsx("col col--4")}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
