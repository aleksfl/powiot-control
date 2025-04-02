import data.data_import as di
import argparse
import logging
import datetime
import argparse
import logging

def setup_logging():
    """Configure logging for the application."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )

def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Basic Python script.")
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser.parse_args()

def main():
    """Main function of the script."""
    args = parse_arguments()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    logging.info("Starting data import...")
    print(di.get_data_from_file(di.file_path, "heatPumpController", "fanLevel", 1, datetime.datetime.fromisoformat("2024-11-13T11:10:00Z"), datetime.datetime.fromisoformat("2024-11-13T11:20:00Z")))
 
    
    logging.info("Finishing data import")

if __name__ == "__main__":
    setup_logging()
    main()