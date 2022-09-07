call plug#begin('~/.vim/bundle')
Plug 'preservim/nerdtree'
Plug 'Yggdroot/LeaderF', { 'do': './install.sh' }
Plug 'tpope/vim-unimpaired'
Plug 'zivyangll/git-blame.vim'
Plug 'pechorin/any-jump.vim'
Plug 'neoclide/coc.nvim', {'branch': 'release'}
call plug#end()

let b:coc_diagnostic_disable = 1
highlight PMenu ctermfg=242 ctermbg=255 guifg=white guibg=white
highlight PMenuSel ctermfg=1 ctermbg=253 guifg=white guibg=white

syntax on
set ts=2
set showmatch
set nocompatible
set incsearch
set clipboard=unnamed
set guicursor=i:block-iCursor-blinkon0,v:block-vCursor
set nrformats=
set history=200
set ignorecase
set smartcase

let mapleader=' '
let g:mapleader=' '
nnoremap <leader>n :NERDTreeFocus<CR>
nnoremap <leader>m :NERDTree<CR>
nnoremap <leader>t :NERDTreeToggle<CR>
nnoremap <leader>f :NERDTreeFind<CR>
nnoremap <leader>q :q<CR>
nnoremap <leader>e :e %:p:s,.h$,.X123X,:s,.cc$,.h,:s,.X123X$,.cc,<CR>
nnoremap <leader>s :<C-u>call gitblame#echo()<CR>
nnoremap <leader>j :AnyJump<CR>
nnoremap <leader>w :w<CR>
nnoremap <C-j> 5j
nnoremap <C-k> 5k
inoremap <silent><expr> <TAB>
      \ coc#pum#visible() ? coc#pum#next(1) :
      \ CheckBackspace() ? "\<Tab>" :
      \ coc#refresh()
inoremap <expr><S-TAB> coc#pum#visible() ? coc#pum#prev(1) : "\<C-h>"
