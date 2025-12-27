import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Comprehensive Learning',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Complete modules covering ROS 2, Digital Twins, AI Robotics, and Vision-Language-Action systems
        designed for engineers and developers.
      </>
    ),
  },
  {
    title: 'Hands-on Approach',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Each module includes practical exercises and quizzes to reinforce learning
        through real-world robotics applications.
      </>
    ),
  },
  {
    title: 'Modern Technologies',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Leverage cutting-edge tools like Gazebo, Unity, NVIDIA Isaac, and LLMs
        for advanced robotics development.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className={clsx('card', styles.featureCard)}>
        <div className="card__body text--center">
          <div className={styles.featureSvgWrapper}>
            <Svg className={styles.featureSvg} role="img" />
          </div>
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container padding-vert--lg">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
