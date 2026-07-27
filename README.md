# irurueta-navigation-inertial

An inertial GNSS/INS navigation library

[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-inertial/actions/workflows/master.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-inertial/actions/workflows/master.yml)
[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-inertial/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-inertial/actions/workflows/develop.yml)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)

## Project Status

| | |
|---|---|
| Language | Java 21+ (compiled with `maven.compiler.source/target` 21; CI builds with JDK 21) |
| Build tool | Maven |
| Current development version | `1.11.0-SNAPSHOT` |
| Latest release | `1.10.0` |
| License | Apache License, Version 2.0 |
| CI | GitHub Actions — build, test, static analysis and deployment on `develop` and `master` |
| Quality | SonarCloud, JaCoCo coverage, Checkstyle, PMD, SpotBugs |

## Documentation

* [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-inertial/) — concepts, installation guide and reference links.
* [Maven Site Report](https://albertoirurueta.github.io/irurueta-navigation-inertial/mvn-site/) — Javadoc, JaCoCo coverage, Surefire test results, Checkstyle, SpotBugs, PMD and JXR cross-referenced source.
* [SonarCloud Dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial)
* [CHANGELOG](CHANGELOG.md)

## Installation

Add the following dependency to your project:

Latest release:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-inertial</artifactId>
    <version>1.10.0</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-inertial</artifactId>
    <version>1.11.0-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

Snapshot artifacts are published from the `develop` branch to the Sonatype snapshots repository; add it as a
repository if you need to resolve snapshot versions:

```xml
<repository>
    <id>sonatype-snapshots</id>
    <url>https://central.sonatype.com/repository/maven-snapshots/</url>
    <releases>
        <enabled>false</enabled>
    </releases>
    <snapshots>
        <enabled>true</enabled>
    </snapshots>
</repository>
```

## How It Works

`irurueta-navigation-inertial` implements strapdown inertial navigation, IMU calibration, and Earth magnetic
field modeling for GNSS/INS integrated navigation systems. It turns raw accelerometer, gyroscope and
magnetometer readings into position, velocity and attitude estimates, and lets those estimates be fused with
GNSS measurements. One of its simplest building blocks is `WMMEarthMagneticFluxDensityEstimator`, which uses the
bundled NOAA World Magnetic Model coefficients to estimate Earth's magnetic flux density at a given location —
useful both as a heading reference and as an input to magnetometer calibration:

```java
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import java.io.IOException;

public class MagneticFieldDemo {
    public static void main(String[] args) throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var latitude = Math.toRadians(41.3874);  // Barcelona
        final var longitude = Math.toRadians(2.1686);

        final NEDMagneticFluxDensity b = estimator.estimate(latitude, longitude);
        System.out.println("N: " + b.getBn() + " T, E: " + b.getBe() + " T, D: " + b.getBd() + " T");
    }
}
```

See the [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-inertial/) for the
full strapdown navigation, calibration and Kalman-filter-based INS/GNSS fusion concepts.

## License

This library is licensed under the [Apache License, Version 2.0](LICENSE.txt).
