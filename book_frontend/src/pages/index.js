import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">🤖 Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">Mastering the Convergence of Artificial Intelligence and Embodied Systems</p>
        <div className="buttons">
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Explore Curriculum
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>ROS 2 Architecture</h3>
              <p>The nervous system of modern robotics</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Digital Twins</h3>
              <p>Virtual replicas for safer, faster development</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>AI-Brain Integration</h3>
              <p>How machines perceive, reason, and act</p>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className="col col--12">
            <div className="text--center padding-horiz--md">
              <h2>Vision-Language-Action Systems</h2>
              <p>Natural human-robot interaction through advanced AI integration</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Educational Book on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}