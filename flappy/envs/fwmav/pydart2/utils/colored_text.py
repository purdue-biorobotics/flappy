from colorama import Fore, Style


def red(txt):
    return Fore.RED + txt + Style.RESET_ALL


def blue(txt):
    return Fore.BLUE + txt + Style.RESET_ALL


def green(txt):
    return Fore.GREEN + txt + Style.RESET_ALL


def yello(txt):
    return Fore.YELLOW + txt + Style.RESET_ALL


def magenta(txt):
    return Fore.MAGENTA + txt + Style.RESET_ALL


def cyan(txt):
    return Fore.CYAN + Style.BRIGHT + txt + Style.RESET_ALL


def bright_red(txt):
    return Fore.RED + Style.BRIGHT + txt + Style.RESET_ALL


def bright_blue(txt):
    return Fore.BLUE + Style.BRIGHT + txt + Style.RESET_ALL


def bright_green(txt):
    return Fore.GREEN + Style.BRIGHT + txt + Style.RESET_ALL


def bright_yello(txt):
    return Fore.YELLOW + Style.BRIGHT + txt + Style.RESET_ALL


def bright_magenta(txt):
    return Fore.MAGENTA + Style.BRIGHT + txt + Style.RESET_ALL


def bright_cyan(txt):
    return Fore.CYAN + Style.BRIGHT + txt + Style.RESET_ALL


def dim_red(txt):
    return Fore.RED + Style.DIM + txt + Style.RESET_ALL


def dim_blue(txt):
    return Fore.BLUE + Style.DIM + txt + Style.RESET_ALL


def dim_green(txt):
    return Fore.GREEN + Style.DIM + txt + Style.RESET_ALL


def dim_yello(txt):
    return Fore.YELLOW + Style.DIM + txt + Style.RESET_ALL


def dim_magenta(txt):
    return Fore.MAGENTA + Style.DIM + txt + Style.RESET_ALL


def dim_cyan(txt):
    return Fore.CYAN + Style.DIM + txt + Style.RESET_ALL
