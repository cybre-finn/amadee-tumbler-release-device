{
  description = "Fast Raspberry Pi Pico C/C++ Development Environment";

  inputs = {
    # Aktuelles Stable Release für maximale Cache-Treffer
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05"; 
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };

        pico-sdk-full = pkgs.pico-sdk.override { withSubmodules = true; };

        # --- 1. Fancy Neovim (Optimiert für Speed) ---
        fancy-nvim = pkgs.wrapNeovim pkgs.neovim-unwrapped {
          configure = {
            customRC = ''
              lua << EOF
              vim.opt.number = true
              vim.opt.relativenumber = true
              vim.opt.shiftwidth = 4
              vim.opt.tabstop = 4
              vim.opt.expandtab = true
              vim.opt.termguicolors = true

              vim.cmd[[colorscheme tokyonight]]
              require('lualine').setup{}

              vim.lsp.config('clangd', {
                cmd = { "clangd", "--background-index", "--compile-commands-dir=./build" }
              })
              vim.lsp.enable('clangd')
              EOF
            '';
            packages.all.start = with pkgs.vimPlugins; [
              nvim-lspconfig
              lualine-nvim
              tokyonight-nvim
              # Nur relevante Grammatiken laden (verhindert stundenlanges Bauen)
              (nvim-treesitter.withPlugins (p: with p; [ c cpp cmake lua bash nix ]))
            ];
          };
        };

        # --- 2. VS Code Setup ---
        custom-vscode = pkgs.vscode-with-extensions.override {
          vscodeExtensions = with pkgs.vscode-extensions; [
            llvm-vs-code-extensions.vscode-clangd
            ms-vscode.cmake-tools
            twxs.cmake
            marus25.cortex-debug
          ];
        };

      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            pkgs.gcc-arm-embedded
            pkgs.cmake
            pkgs.ninja
            pkgs.python3
            pkgs.clang-tools
            pkgs.picotool
            
            # Das SDK kommt jetzt extrem schnell direkt aus dem Binary Cache von nixpkgs!
            pico-sdk-full
            
            custom-vscode
            fancy-nvim
            
            pkgs.zsh
            pkgs.starship
          ];

          # Nixpkgs installiert das Pico SDK unter diesem Pfad
          PICO_SDK_PATH = "${pico-sdk-full}/lib/pico-sdk";

          shellHook = ''
            if [ -z "$IN_PICO_SHELL" ]; then
              export IN_PICO_SHELL=1
              export ZDOTDIR=$(mktemp -d)
              
              cat << 'EOF' > $ZDOTDIR/.zshrc
              autoload -Uz compinit && compinit
              eval "$(${pkgs.starship}/bin/starship init zsh)"

              echo -e "\033[1;34m=================================================================\033[0m"
              echo -e "\033[1;32m 🚀 Pico Development Environment loaded (Stable & Fast)! \033[0m"
              echo -e "\033[1;34m=================================================================\033[0m"
              echo ""
              echo -e "\033[1;33m🛠️  Clangd Setup (VS Code & Nvim):\033[0m"
              echo -e "   \033[1;36m👉 cmake -B build -S . -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\033[0m"
              echo ""
              echo -e "\033[1;33m⚡ Flashing Commands (picotool via USB):\033[0m"
              echo -e "   1. \033[1;36mcmake --build build\033[0m                        (Projekt kompilieren)"
              echo -e "   2. \033[1;36mpicotool load build/*.uf2 -f -x\033[0m          (Flashen & direkt Starten)"
              echo -e "   3. \033[1;36mpicotool reboot\033[0m                          (Nur Neustart erzwingen)"
              echo -e "\033[1;34m=================================================================\033[0m"
              
              alias flash="picotool load build/*.uf2 -f -x"
              EOF
              
              exec zsh
            fi
          '';
        };
      }
    );
}
