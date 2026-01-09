#!/bin/bash

# Sync logs from remote server to local
# Usage: ./sync_logs.sh [options]


# Variables need to be set in the "~/.loco_lab_env" file, these should be read from the environment variables
# REMOTE_SERVER_IP="10.10.10.10"
# REMOTE_SERVER_USER="username"
# REMOTE_SERVER_PORT="22"
# REMOTE_LOG_ROOT_PATH="/home/$REMOTE_SERVER_USER/loco_lab/logs/rsl_rl/"

# Variables as script parameters
REMOTE_LOG_TASK="go2_flat"
REMOTE_LOG_RUN="2025-10-10-00-00"
REMOTE_LOG_CHECKPOINT="10000"
ONLY_CHECKPOINT=false
CONFIG_FILE_NAME=".loco_lab_env"

# Variables do not change
LOCAL_LOG_ROOT_PATH="$(cd "$(dirname "$0")"/.. && pwd)/logs/rsl_rl"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Show help information
show_help() {
    echo "Sync logs from remote server to local"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -s, --server SERVER        Server address (example: 10.10.10.10)"
    echo "  -u, --user USER            Remote username (example: username)"
    echo "  -P, --port PORT            SSH port (example: 22)"
    echo "  -p, --path LOG_ROOT_PATH   Remote log root path (example: /home/username/loco_lab/logs/rsl_rl/)"
    echo "  -c, --checkpoint CHECKPOINT_NAME     Specify checkpoint name (default: $REMOTE_LOG_CHECKPOINT)"
    echo "  -t, --task TASK_NAME       Specify task name (default: $REMOTE_LOG_TASK)"
    echo "  -r, --run RUN_NAME         Specify run name (default: $REMOTE_LOG_RUN)"
    echo "  -oc, --only-checkpoint     Only sync checkpoint files, skip auxiliary files"
    echo "  -h, --help                 Show this help information"
    echo ""
    echo "Examples:"
    echo "  $0 -s 10.10.10.10 -u myuser -t unitree_go2_flat -r 2025-10-21 -c 10000"
    echo "  $0 -t unitree_go2_flat -r 2025-10-21 -c 10000"
}

# Log functions
log_info() {
    echo -e "${BLUE}[INFO] $1 ${NC}"
}

log_success() {
    echo -e "${GREEN}[SUCCESS] $1 ${NC}"
}

log_warning() {
    echo -e "${YELLOW}[WARNING] $1 ${NC}"
}

log_error() {
    echo -e "${RED}[ERROR] $1 ${NC}"
}

log_normal() {
    echo -e "${NC}$1 ${NC}"
}

# Clean up empty directories
cleanup_empty_directories() {
    local local_log_path="$1"
    local local_run_path="$local_log_path/$REMOTE_LOG_RUN"

    # Check if the run directory is empty and remove it
    if [[ -d "$local_run_path" ]] && [[ -z "$(ls -A "$local_run_path" 2>/dev/null)" ]]; then
        log_info "===== Removing empty directory: $local_run_path ====="
        rmdir "$local_run_path"
    fi

    # Check if the task directory is empty and remove it
    if [[ -d "$local_log_path" ]] && [[ -z "$(ls -A "$local_log_path" 2>/dev/null)" ]]; then
        log_info "===== Removing empty directory: $local_log_path ====="
        rmdir "$local_log_path"
    fi
}

# SCP command for checkpoints
scp_checkpoints() {
    local local_log_path="$1"
    local remote_checkpoint_path="$REMOTE_LOG_ROOT_PATH/$REMOTE_LOG_TASK/$REMOTE_LOG_RUN/model_$REMOTE_LOG_CHECKPOINT.pt"
    local local_checkpoint_path="$local_log_path/model_$REMOTE_LOG_CHECKPOINT.pt"
    log_normal " Starting to copy files ... "
    log_normal "     from server: '$remote_checkpoint_path' "
    log_normal "     to local: '$local_checkpoint_path' "

    # Build scp command with optional port
    local scp_cmd="scp"
    if [[ -n "$REMOTE_SERVER_PORT" ]]; then
        scp_cmd="scp -P $REMOTE_SERVER_PORT"
    fi

    if $scp_cmd "$REMOTE_SERVER_USER@$REMOTE_SERVER_IP:$remote_checkpoint_path" "$local_checkpoint_path"; then
        log_success "===== Copied model_$REMOTE_LOG_CHECKPOINT.pt to $local_log_path ====="
    else
        log_error "===== Failed to copy checkpoint model_$REMOTE_LOG_CHECKPOINT.pt ====="
        log_info "===== Cleaning up empty directories ====="
        cleanup_empty_directories "$local_log_path"
        return 1
    fi
}

# SCP command for others
scp_others() {
    local local_log_path="$1"
    local remote_run_path="$REMOTE_LOG_ROOT_PATH/$REMOTE_LOG_TASK/$REMOTE_LOG_RUN"
    local local_run_path="$local_log_path/$REMOTE_LOG_RUN"
    local any_files_copied=false

    # Build scp command with optional port
    local scp_cmd="scp"
    if [[ -n "$REMOTE_SERVER_PORT" ]]; then
        scp_cmd="scp -P $REMOTE_SERVER_PORT"
    fi

    # Create local run directory if it doesn't exist
    if [[ ! -d "$local_run_path" ]]; then
        mkdir -p "$local_run_path"
    fi

    # Copy git folder
    log_normal " Copying 'git' folder ..."
    if $scp_cmd -r "$REMOTE_SERVER_USER@$REMOTE_SERVER_IP:$remote_run_path/git" "$local_run_path/"; then
        log_success "===== Copied 'git' folder ====="
        any_files_copied=true
    else
        log_warning "===== Failed to copy 'git' folder (may not exist) ====="
    fi

    # Copy params folder
    log_normal " Copying 'params' folder ..."
    if $scp_cmd -r "$REMOTE_SERVER_USER@$REMOTE_SERVER_IP:$remote_run_path/params" "$local_run_path/"; then
        log_success "===== Copied 'params' folder ====="
        any_files_copied=true
    else
        log_warning "===== Failed to copy 'params' folder (may not exist) ====="
    fi

    # Copy events files
    log_normal " Copying events files ..."
    if $scp_cmd "$REMOTE_SERVER_USER@$REMOTE_SERVER_IP:$remote_run_path/events.out.tfevents.*" "$local_run_path/" 2>/dev/null; then
        log_success "===== Copied events files ====="
        any_files_copied=true
    else
        log_warning "===== No tensorboard files found or failed to copy ====="
    fi

    # If no files were copied, clean up empty directories
    if [[ "$any_files_copied" == false ]]; then
        log_info "===== No auxiliary files were copied, cleaning up empty directories ====="
        cleanup_empty_directories "$local_log_path"
    fi
}

# Check presence of environment variables
check_environment_variables() {
    local is_error=false

    # Check if required variables are set (either from environment or command line)
    if [[ -z "$REMOTE_SERVER_IP" ]]; then
        log_error "===== REMOTE_SERVER_IP is not set (use -s/--server or set in ~/$CONFIG_FILE_NAME) ====="
        is_error=true
    fi
    if [[ -z "$REMOTE_SERVER_USER" ]]; then
        log_error "===== REMOTE_SERVER_USER is not set (use -u/--user or set in ~/$CONFIG_FILE_NAME) ====="
        is_error=true
    fi
    if [[ -z "$REMOTE_LOG_ROOT_PATH" ]]; then
        log_error "===== REMOTE_LOG_ROOT_PATH is not set (use -p/--path or set in ~/$CONFIG_FILE_NAME) ====="
        is_error=true
    fi
    if $is_error; then
        exit 1
    fi
}

# Prepare local path
check_local_path() {
    if [[ ! -d "$LOCAL_LOG_ROOT_PATH" ]]; then
        log_info "===== Creating local directory: $LOCAL_LOG_ROOT_PATH ====="
        mkdir -p "$LOCAL_LOG_ROOT_PATH"
    fi
    local local_log_path="$LOCAL_LOG_ROOT_PATH/$REMOTE_LOG_TASK/$REMOTE_LOG_RUN"
    if [[ ! -d "$local_log_path" ]]; then
        log_info "===== Creating local directory: $local_log_path ====="
        mkdir -p "$local_log_path"
    fi
}

# Parse command line arguments
parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -s|--server)
                REMOTE_SERVER_IP="$2"
                shift 2
                ;;
            -u|--user)
                REMOTE_SERVER_USER="$2"
                shift 2
                ;;
            -P|--port)
                REMOTE_SERVER_PORT="$2"
                shift 2
                ;;
            -p|--path)
                REMOTE_LOG_ROOT_PATH="$2"
                shift 2
                ;;
            -c|--checkpoint)
                REMOTE_LOG_CHECKPOINT="$2"
                shift 2
                ;;
            -t|--task)
                REMOTE_LOG_TASK="$2"
                shift 2
                ;;
            -r|--run)
                REMOTE_LOG_RUN="$2"
                shift 2
                ;;
            -oc|--only-checkpoint)
                ONLY_CHECKPOINT=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# SCP logs
main() {
    # Parse command line arguments first
    parse_arguments "$@"

    # Load environment variables (command line args will override these)
    if [[ -f "$HOME/$CONFIG_FILE_NAME" ]]; then
        source "$HOME/$CONFIG_FILE_NAME"
        log_info "===== Environment variables loaded from $HOME/$CONFIG_FILE_NAME ====="
    fi

    # check environment variables (now includes command line overrides)
    check_environment_variables

    # check local path
    check_local_path

    # sync checkpoints
    log_info "===== Starting checkpoint sync ====="
    if ! scp_checkpoints "$LOCAL_LOG_ROOT_PATH/$REMOTE_LOG_TASK/$REMOTE_LOG_RUN"; then
        exit 1
    fi

    # sync others (only if not --only-checkpoint)
    if [[ "$ONLY_CHECKPOINT" == false ]]; then
        log_info "===== Starting auxiliary files sync ====="
        if ! scp_others "$LOCAL_LOG_ROOT_PATH/$REMOTE_LOG_TASK"; then
            exit 1
        fi
    else
        log_info "===== Skipping auxiliary files sync (--only-checkpoint specified) ====="
    fi

    # log success
    echo ""
    log_success "===== Sync logs completed successfully ====="
}

# Run main function
main "$@"
