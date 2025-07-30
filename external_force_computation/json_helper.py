import json
from pathlib import Path
from typing import Union, List, Dict


def parse_json(json_file: Union[str, Path]) -> Dict:
    """
    Parses a JSON file and returns its content as a dictionary.

    Args:
        json_file (Union[str, Path]): Path to the JSON file.

    Returns:
        Dict: Parsed JSON content.

    Raises:
        json.JSONDecodeError: If the file contains invalid JSON.
        FileNotFoundError: If the file does not exist.
        ValueError: If an unknown issue occurs during reading.
    """
    json_file = Path(json_file)
    
    try:
        with json_file.open("r", encoding="utf-8") as file:
            data = json.load(file)
        return data
    except (json.JSONDecodeError, FileNotFoundError, ValueError) as error:
        raise error


def load_directory(directory: Union[str, Path]) -> List[Dict]:
    """
    Loads all JSON files from a given directory and returns their contents as a list.

    Args:
        directory (Union[str, Path]): Path to the directory containing JSON files.

    Returns:
        List[Dict]: A list of dictionaries, each representing the contents of a JSON file.

    Raises:
        FileNotFoundError: If the directory does not exist.
        ValueError: If the directory contains no JSON files.
    """
    directory = Path(directory)
    
    if not directory.exists() or not directory.is_dir():
        raise FileNotFoundError(f"Directory '{directory}' does not exist or is not a directory.")
    
    json_files = sorted(directory.glob("*.json"))  # Get all JSON files sorted by name

    if not json_files:
        raise ValueError(f"No JSON files found in directory '{directory}'.")

    data_list = [parse_json(file) for file in json_files]

    return data_list


def save_data_to_json(filepath: Union[Path, str], filename: Union[Path, str], **kwargs) -> None:
    """
    Save data to a JSON file in a specified file.
    """
    filename = Path(filename)
    if not filename.suffix == ".json":
        filename = filename.with_suffix('.json')

    target_file = filename if filename.parent != Path('.') else Path(filepath) / filename
    data_to_save = {"filename": filename.name}
    data_to_save.update(kwargs)

    with target_file.open("w") as f:
        json.dump(data_to_save, f, indent=4)
