# Position Correction Tool

## Overview

The **Position Correction Tool** is a Python-based application designed to process and correct geospatial data, particularly focusing on the East (E) coordinates, height (H), and yaw (orientation) of georeferenced images. This tool is tailored for applications such as solar plant inspections, where accurate positioning is crucial.

## Features

- **East Coordinate Correction (`correctE.py`)**: Adjusts and validates East coordinates in geospatial systems, correcting common errors to enhance data accuracy.
- **Height Correction (`correctH.py`)**: Processes geospatial data and images to calculate distances, analyze geometric points, and manipulate images for height adjustments.
- **Yaw Correction (`correctYaw.py`)**: Processes georeferenced images to calculate distances between geographic points, order points, and correct the orientation (yaw) of images based on specific parameters and metadata.
- **Automated Processing (`corrector.py`)**: Serves as the core script integrating multiple correction scripts and analytical tools, facilitating automatic adjustments based on the specific configuration of each solar plant.

## Repository Structure

- `correctScripts/`: Contains scripts for correcting East coordinates, height, and yaw.
- `kmlTables/`: Stores KML files and related data tables.
- `oldcorrectScripst/`: Archive of previous versions of correction scripts.
- `.gitignore`: Specifies files and directories to be ignored by Git.
- `README.md`: This file.
- `corrector.py`: Main script orchestrating the correction processes.
- `requirements.txt`: Lists Python dependencies required to run the project.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mgnavarrete/positionCorrect.git
   cd positionCorrect
   ```

2. **Create a Virtual Environment** (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. **Prepare Data**: Ensure that your georeferenced images and metadata are organized in the appropriate directories as expected by the scripts.

2. **Run the Main Script**:
   ```bash
   python corrector.py
   ```

   Follow the on-screen prompts to select the specific solar plant configuration you wish to process. The available options include:

   - Finis Terrae (FIT)
   - Finis Terrae Extension (FIX)
   - Campos del Sol (CDS)
   - Lalakama (LLK)
   - Sol de Lila (SDL)

3. **Review Results**: Corrected metadata and any generated reports or data will be available in the output directories specified within the scripts.

## Contributing

Contributions are welcome! If you have suggestions for improvements or new features, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
